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
    def __init__(self, save_dir, rate=50, start_loop=False):
        """
        Records joint data to a file at a specified rate.
        rate: recording frequency in Hertz
        :param save_dir  where to save the recordings
        :param rate
        :param start_loop whether to start recording in a loop
        :param whether the recorder instance is an auxiliary recorder
        """
        side = "right"

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


        rospy.sleep(0.5)

        self.bridge = CvBridge()

        # only for testing  !!!!!!!!!!!!!!!!!!!!!!!!
        # self.start_loop = start_loop
        # self.init_traj(itr=0)

        self.ngroup = 500
        self.igrp = 0

        # if it is an auxiliary node advertise services
        if self.instance_type != 'main':
            rospy.init_node('aux_recorder1')
            rospy.Service('get_kinectdata', get_kinectdata, self.get_kinect_handler)
            rospy.Service('init_traj', init_traj, self.init_traj_handler)
            rospy.spin()
        else:
            def spin_thread():
                rospy.spin()

            pdb.set_trace()
            thread.start_new(spin_thread, ())
            print "Recorder intialized."
            print "started spin thread"


    def get_kinect_handler(self, req):
        self._save_local()
        print 'started service handler'
        return get_kinectdataResponse(self.ltob.img_msg, self.ltob.d_img_msg)

    def init_traj_handler(self, req):
        self._init_traj_local(req.itr)

    def store_latest_d_im(self, data):
        self.ltob.d_img_msg = data
        cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')

        self.ltob.d_img_raw_npy = np.asarray(cv_image)
        img = cv2.resize(cv_image, (0, 0), fx=1 /5.5, fy=1 / 5.5)

        # print '----------------------'
        # print 'image raw data:'
        # print 'depth image shape', img.shape
        # print 'max', np.max(img)
        # print 'min', np.min(img)
        # print '----------------------'
        self.ltob.tstamp_d_img = rospy.get_time()

        # percentile = np.percentile(num, 90)
        # print '90-th percentile at', percentile
        # ## fixing this at 1400
        img = np.clip(img,0, 1400)

        startcol = 14
        startrow = 0
        endcol = startcol + 64
        endrow = startrow + 64
        #crop image:
        img = img[startrow:endrow, startcol:endcol]

        self.ltob.d_img_cropped_npy = img
        img = img.astype(np.float32)/ np.max(img) *256
        img = img.astype(np.uint8)
        img = np.squeeze(img)
        self.ltob.d_img_cropped_8bit = Image.fromarray(img)

        # Image.fromarray(img).show()
        # if self.start_loop:
        #     self.save()


    def store_latest_im(self, data):
        self.ltob.img_msg = data
        self.ltob.tstamp_img = rospy.get_time()
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  #(1920, 1080)

        self.ltob.img_cv2 =  cv_image
        self.ltob.img_cropped = self.crop_colorimg(cv_image)

        # cv_image.imshow()
        # pdb.set_trace()
        # small = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)
        # small.imshow()
        # pdb.set_trace()


    def crop_colorimg(self, cv_image):
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(cv_image)
        img.thumbnail(np.asarray(img.size) / 14, Image.ANTIALIAS)

        startcol = 34
        startrow = 3
        endcol = startcol + 64
        endrow = startrow + 64
        img = img.crop((startcol, startrow, endcol, endrow))

        # print 'cropped'
        # img.show()
        # pdb.set_trace()
        return img


    def set_igrp(self, igrp):
        self.igrp = igrp


    def init_traj(self, itr):
        if self.instance_type == 'main':
            # request init service for auxiliary recorders
            rospy.loginfo("waiting for service init_traj...")
            rospy.wait_for_service('init_traj')
            init_traj_func = rospy.ServiceProxy('init_traj', init_traj)
            resp1 = init_traj_func()
            rospy.loginfo("init srv call succeeded")

        self._init_traj_local(itr)


    def _init_traj_local(self, itr):
        """
        :param itr: number of current trajecotry
        :return:
        """

        if ((itr+1) % self.ngroup) == 0 or self.igrp == 0:
            self.igrp += 1
            group_folder = self.save_dir + '/traj_group{}'.format(self.igrp)


        rospy.loginfo("Init trajectory {} in group {}".format(itr, self.igrp))

        traj_folder = group_folder + '/traj{}'.format(itr)
        self.image_folder = traj_folder + '/images'
        self.depth_image_folder = traj_folder + '/depth_images'

        if not os.path.exists(traj_folder):
            os.makedirs(traj_folder)
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
        if not os.path.exists(self.depth_image_folder):
            os.makedirs(self.depth_image_folder)

        if self.instance_type == 'main':
            self.joint_data_file = traj_folder + '/joint_angles_traj{}'.format(itr)
            joints_right = self._limb_right.joint_names()
            with open(self.joint_data_file, 'w+') as f:
                f.write('time,')
                temp_str = '' if self._gripper else '\n'
                f.write(','.join([j for j in joints_right]) + ',' + temp_str)

    def delete(self, i_tr):
        traj_folder = self.save_dir + '/traj{}'.format(i_tr)
        shutil.rmtree(traj_folder)
        print 'deleted {}'.format(traj_folder)


    def save(self, i_tr):
        if self.instance_type == 'main':
            # request init service for auxiliary recorders
            rospy.loginfo("waiting for service get_kinectdata...")
            rospy.wait_for_service('get_kinectdata')
            get_kinectdata_func = rospy.ServiceProxy('get_kinectdata', get_kinectdata)
            resp1 = get_kinectdata_func()
            rospy.loginfo("get_kinectdata srv call succeeded")

        self._save_local()

    def _save_local(self, i_tr = None):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """

        if self.instance_type == 'main':

            joints_right = self._limb_right.joint_names()
            with open(self.joint_data_file, 'a') as f:
                temp_str = '' if self._gripper else '\n'
                angles_right = [self._limb_right.joint_angle(j)
                                for j in joints_right]
                f.write("%f," % (self._time_stamp(),))
                f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)

        pref = self.instance_type
        #saving image
        # saving the full resolution image
        image_name =  self.image_folder+ "/" + pref + "_full_im{0}_time{1}.jpg".format(i_tr, self.ltob.tstamp_img)
        cv2.imwrite(image_name, self.ltob.img_cv2, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

        # saving the cropped and downsized image
        image_name = self.image_folder + "/" + pref +"_cropped_im{0}_time{1}.png".format(i_tr, self.ltob.tstamp_img)
        self.ltob.img_cropped.save(image_name, "PNG")

        # saving the cropped depth data in a Pickle file
        file = self.depth_image_folder + "/" + pref +"_depth_im{0}_time{1}.pkl".format(i_tr, self.ltob.tstamp_d_img)
        cPickle.dump(self.ltob.d_img_cropped_npy, open(file, 'wb'))

        # saving downsampled 8bit images
        image_name = self.depth_image_folder + "/" + pref + "_cropped_depth_im{0}_time{1}.png".format(i_tr, self.ltob.tstamp_d_img)
        self.ltob.d_img_cropped_8bit.save(image_name, "PNG")
        pdb.set_trace()


if __name__ ==  '__main__':
    print 'started'
    rec = RobotRecorder('/home/guser/Documents/sawyer_data/testrecording')