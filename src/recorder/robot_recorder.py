#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as Image_msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import shutil
import socket
import thread
import numpy as np
import pdb
from berkeley_sawyer.srv import *
from PIL import Image
import cPickle


class Latest_observation(object):
    def __init__(self):
        #color image:
        self.img_cv2 = None
        self.img_cropped = None
        self.tstamp_img = None  # timestamp of image
        self.img_msg = None

        #depth image:
        self.d_img_raw_npy = None  # 16 bit raw data
        self.d_img_cropped_npy = None
        self.d_img_cropped_8bit = None
        self.tstamp_d_img = None  # timestamp of image
        self.d_img_msg = None


class RobotRecorder(object):
    def __init__(self, save_dir, rate=50, start_loop=False, seq_len = None):
        """
        Records joint data to a file at a specified rate.
        rate: recording frequency in Hertz
        :param save_dir  where to save the recordings
        :param rate
        :param start_loop whether to start recording in a loop
        :param whether the recorder instance is an auxiliary recorder
        """
        side = "right"
        self.state_sequence_length = seq_len
        self.overwrite = True

        rospy.loginfo('hostname is :{}'.format(socket.gethostname()))
        if socket.gethostname() != 'kullback':
            # if node is not running on kullback it is an auxiliary recorder
            self.instance_type = 'aux1'
        else:
            # instance running on kullback is called main;
            # them main instance one also records actions and joint angles
            self.instance_type = 'main'


        if self.instance_type =='main': #if it is running on kullback
            self._gripper = None
            self.gripper_name = '_'.join([side, 'gripper'])
            import intera_interface
            self._limb_right = intera_interface.Limb(side)


        prefix = self.instance_type

        rospy.Subscriber(prefix + "/kinect2/hd/image_color", Image_msg, self.store_latest_im)
        rospy.Subscriber(prefix + "/kinect2/sd/image_depth_rect", Image_msg, self.store_latest_d_im)


        self.save_dir = save_dir
        self.ltob = Latest_observation()

        self.bridge = CvBridge()

        self.ngroup = 1000
        self.igrp = 0

        # if it is an auxiliary node advertise services
        if self.instance_type != 'main':
            rospy.init_node('aux_recorder1')
            rospy.loginfo("init node aux_recorder1")

            # initializing the server:
            rospy.Service('get_kinectdata', get_kinectdata, self.get_kinect_handler)
            rospy.Service('init_traj', init_traj, self.init_traj_handler)
            rospy.Service('delete_traj', delete_traj, self.delete_traj_handler)
            rospy.spin()
        else:
            # initializing the client:
            self.get_kinectdata_func = rospy.ServiceProxy('get_kinectdata', get_kinectdata)
            self.init_traj_func = rospy.ServiceProxy('init_traj', init_traj)
            self.delete_traj_func = rospy.ServiceProxy('delete_traj', delete_traj)

            def spin_thread():
                rospy.spin()
            thread.start_new(spin_thread, ())
            print "Recorder intialized."
            print "started spin thread"
            self.action_list, self.joint_angle_list = [], []


    def get_kinect_handler(self, req):
        self.t_savereq = rospy.get_time()
        self._save_img_local(req.itr)
        return get_kinectdataResponse(self.ltob.img_msg, self.ltob.d_img_msg)

    def init_traj_handler(self, req):
        self.igrp = req.igrp
        self._init_traj_local(req.itr)
        return init_trajResponse()

    def delete_traj_handler(self, req):
        self._delete_traj_local(req.itr)
        return delete_trajResponse()

    def store_latest_d_im(self, data):
        # if self.ltob.tstamp_img != None:
            # rospy.loginfo("time difference to last stored dimg: {}".format(
            #     rospy.get_time() - self.ltob.tstamp_d_img
            # ))

        self.ltob.tstamp_d_img = rospy.get_time()

        self.ltob.d_img_msg = data
        cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')

        self.ltob.d_img_raw_npy = np.asarray(cv_image)
        img = cv2.resize(cv_image, (0, 0), fx=1 /5.5, fy=1 / 5.5, interpolation=cv2.INTER_AREA)

        # print '----------------------'
        # print 'image raw data:'
        # print 'depth image shape', img.shape
        # print 'max', np.max(img)
        # print 'min', np.min(img)
        # print '----------------------'

        # percentile = np.percentile(num, 90)
        # print '90-th percentile at', percentile
        # ## fixing this at 1400
        img = np.clip(img,0, 1400)

        startcol = 7
        startrow = 0
        endcol = startcol + 64
        endrow = startrow + 64
        #crop image:
        img = img[startrow:endrow, startcol:endcol]

        self.ltob.d_img_cropped_npy = img
        img = img.astype(np.float32)/ np.max(img) *256
        img = img.astype(np.uint8)
        img = np.squeeze(img)
        self.ltob.d_img_cropped_8bit = img

        # Image.fromarray(img).show()
        # pdb.set_trace()

        # rospy.loginfo("time to save depth-image: {}".format(
        #     rospy.get_time() - self.ltob.tstamp_d_img
        # ))

    def store_latest_im(self, data):
        # rospy.loginfo('storing color image')
        # if self.ltob.tstamp_img != None:
        #     rospy.loginfo("time difference to last stored img: {}".format(
        #         rospy.get_time() - self.ltob.tstamp_img
        #     ))

        self.ltob.img_msg = data
        self.ltob.tstamp_img = rospy.get_time()
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  #(1920, 1080)

        self.ltob.img_cv2 =  cv_image
        self.ltob.img_cropped = self.crop_colorimg(cv_image)

        # rospy.loginfo("time to save image: {}".format(
        #     rospy.get_time() - self.ltob.tstamp_img
        # ))


    def crop_colorimg(self, cv_image):
        self.ltob.d_img_raw_npy = np.asarray(cv_image)
        img = cv2.resize(cv_image, (0, 0), fx=1 / 15., fy=1 / 15., interpolation=cv2.INTER_AREA)

        startcol = 27
        startrow = 2
        endcol = startcol + 64
        endrow = startrow + 64
        # crop image:
        img = img[startrow:endrow, startcol:endcol]

        # cv2.imshow('img', img)
        # print 'cropped'
        # pdb.set_trace()
        return img


    def set_igrp(self, igrp):
        self.igrp = igrp


    def init_traj(self, itr):
        assert self.instance_type == 'main'
        # request init service for auxiliary recorders

        try:
            rospy.wait_for_service('init_traj', timeout=0.1)
            resp1 = self.init_traj_func(itr, self.igrp)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            raise ValueError('get_kinectdata service failed')

        self._init_traj_local(itr)

        if ((itr+1) % self.ngroup) == 0:
            self.igrp += 1


    def _init_traj_local(self, itr):
        """
        :param itr: number of current trajecotry
        :return:
        """


        self.group_folder = self.save_dir + '/traj_group{}'.format(self.igrp)

        rospy.loginfo("Init trajectory {} in group {}".format(itr, self.igrp))


        traj_folder = self.group_folder + '/traj{}'.format(itr)
        self.image_folder = traj_folder + '/images'
        self.depth_image_folder = traj_folder + '/depth_images'

        if not os.path.exists(traj_folder):
            os.makedirs(traj_folder)
        else:
            if not self.overwrite:
                raise ValueError("trajectory {} already exists".format(traj_folder))
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
        if not os.path.exists(self.depth_image_folder):
            os.makedirs(self.depth_image_folder)

        if self.instance_type == 'main':
            self.state_action_data_file = traj_folder + '/joint_angles_traj{}.txt'.format(itr)
            self.state_action_pkl_file = traj_folder + '/joint_angles_traj{}.pkl'.format(itr)
            joints_right = self._limb_right.joint_names()
            with open(self.state_action_data_file, 'w+') as f:
                f.write('time,')
                action_names = ['move','val_move_x','val_move_y','close','val_close','up','val_up']
                captions = joints_right + action_names
                f.write(','.join(captions) + ',' + '\n')

    def _save_state_actions(self, i_tr, action):
        joints_right = self._limb_right.joint_names()
        with open(self.state_action_data_file, 'a') as f:
            angles_right = [self._limb_right.joint_angle(j)
                            for j in joints_right]
            f.write("%f," % (rospy.get_time(),))
            values = np.concatenate([angles_right, action])
            f.write(','.join([str(x) for x in values]) + '\n')

        self.joint_angle_list.append(angles_right)
        self.action_list.append(action)

        if i_tr == self.state_sequence_length-1:
            joint_angles = np.stack(self.joint_angle_list)
            actions = np.stack(self.action_list)

            with open(self.state_action_pkl_file, 'wb') as f:
                dict= {'jointangles': joint_angles,
                       'actions': actions}
                cPickle.dump(dict, f)

    def delete_traj(self, tr):
        assert self.instance_type == 'main'
        try:
            rospy.wait_for_service('delete_traj', 0.1)
            resp1 = self.delete_traj_func(tr)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            raise ValueError('delete traj service failed')
        self._delete_traj_local(tr)

    def _delete_traj_local(self, i_tr):
        self.group_folder = self.save_dir + '/traj_group{}'.format(self.igrp)
        traj_folder = self.group_folder + '/traj{}'.format(i_tr)
        shutil.rmtree(traj_folder)
        print 'deleted {}'.format(traj_folder)

    def save(self, i_tr, action, endeffector_pose):
        self.t_savereq = rospy.get_time()
        assert self.instance_type == 'main'

        # request save at auxiliary recorders
        try:
            # t1 = rospy.get_time()
            rospy.wait_for_service('get_kinectdata', 0.1)
            #rospy.loginfo("t waiting for service {}".format(rospy.get_time() - t1))
            # t2 = rospy.get_time()
            resp1 = self.get_kinectdata_func(i_tr)
            #rospy.loginfo("t calling service {}".format(rospy.get_time() - t2))
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            raise ValueError('get_kinectdata service failed')

        #rospy.loginfo("time to complete service {}".format(rospy.get_time()- self.t_savereq))


        t_beforesave = rospy.get_time()
        try:
            self._save_img_local(i_tr)
            self._save_state_actions(i_tr, action, endeffector_pose)
        except ValueError:
            return

        #rospy.loginfo("complete time to save locally {}".format(rospy.get_time() - t_beforesave))

        # timing statistics:

        # d_times = [delta_req_store_dimage, delta_req_store_image, complete_time_save]
        # if not (d_times < 0.4).all():
        #     raise ValueError("images could not be captured in time!")

    def _save_img_local(self, i_tr):

        pref = self.instance_type
        #saving image
        # saving the full resolution image
        if self.ltob.img_cv2 is not None:
            image_name =  self.image_folder+ "/" + pref + "_full_im{0}_time{1}.jpg".format(i_tr, self.ltob.tstamp_img)
            cv2.imwrite(image_name, self.ltob.img_cv2, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        else:
            raise ValueError('img_cv2 no data received')

        # saving the cropped and downsized image
        if self.ltob.img_cropped is not None:
            image_name = self.image_folder + "/" + pref +"_cropped_im{0}_time{1}.png".format(i_tr, self.ltob.tstamp_img)
            cv2.imwrite(image_name, self.ltob.img_cropped, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT,1])
        else:
            raise ValueError('img_cropped no data received')

        # saving the depth data
        # saving the cropped depth data in a Pickle file
        if self.ltob.d_img_cropped_npy is not None:
            file = self.depth_image_folder + "/" + pref +"_depth_im{0}_time{1}.pkl".format(i_tr, self.ltob.tstamp_d_img)
            cPickle.dump(self.ltob.d_img_cropped_npy, open(file, 'wb'))
        else:
            raise ValueError('d_img_cropped_npy no data received')

        # saving downsampled 8bit images
        if self.ltob.d_img_cropped_8bit is not None:
            image_name = self.depth_image_folder + "/" + pref + "_cropped_depth_im{0}_time{1}.png".format(i_tr, self.ltob.tstamp_d_img)
            cv2.imwrite(image_name, self.ltob.d_img_cropped_8bit, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT, 1])
        else:
            raise ValueError('d_img_cropped_8bit no data received')

        # rospy.loginfo("time of request {}".format(self.t_savereq))
        # delta_req_store_dimage = self.t_savereq - self.ltob.tstamp_d_img
        # rospy.loginfo("time between last stored depthimage and save request: {}"
        #               .format(delta_req_store_dimage))
        # delta_req_store_image = self.t_savereq - self.ltob.tstamp_img
        # rospy.loginfo("time between last stored image and save request: {}"
        #               .format(delta_req_store_image))
        # complete_time_save = rospy.get_time() - self.t_savereq
        # rospy.loginfo("complete time for saving: {}"
        #               .format(complete_time_save))


if __name__ ==  '__main__':
    print 'started'
    rec = RobotRecorder('/home/guser/Documents/sawyer_data/testrecording')