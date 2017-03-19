#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import os
import shutil
import pdb

import thread

class KinectRecorder(object):
    def __init__(self, save_dir, rate=10, start_loop=True):
        """
        kinectdata with specified rate
        rate: recording frequency in Hertz
        """

        print("Initializing node... ")
        rospy.init_node("kinect_data_collector_node")
        rospy.on_shutdown(self.clean_shutdown)

        self._do_recording = False
        self.save_dir = save_dir
        self.start_loop = start_loop  # whether to create a loop for saving or not

        self.latest_image = None
        self.latest_depth_image_raw = None
        self.time_stamp_im = None  #timestamp of image
        self.time_stamp_d_im = None  # timestamp of image


        rospy.Subscriber("/kinect2/hd/image_color", Image, self.store_latest_im)
        # rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.store_latest_d_im)

        # rospy.Subscriber("/kinect2/sd/image_color_rect", Image, self.store_latest_im)
        self.dsubs = rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, self.store_latest_d_im)
        self.bridge = CvBridge()
        print "Recorder intialized."

        if not start_loop:
            def spin_thread():
                print "started spin thread"
                rospy.spin()

            thread.start_new(spin_thread, ())

        else:
            self.init_traj(0)
            rospy.spin()

    def store_latest_im(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.latest_image = cv_image
        self.time_stamp_im = self._time_stamp()
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        # if self.start_loop:
        #     self.save()

    def store_latest_d_im(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')
        self.latest_depth_image_raw = num = np.asarray(cv_image)
        self.time_stamp_d_im = self._time_stamp()

        # percentile = np.percentile(num, 90)
        # print '90-th percentile at', percentile
        # ## fixing this at 1400
        num = np.clip(num,0, 1400)
        num = num.astype(np.float32)/ np.max(num) *256
        num = num.astype(np.uint8)

        num = np.squeeze(num)
        print num.shape
        print np.max(num)
        print np.min(num)

        from PIL import Image
        Image.fromarray(num).show()

        pdb.set_trace()

        if self.start_loop:
            self.save()

    def _time_stamp(self):
        return rospy.get_time()

    def init_traj(self, i_tr):
        """
        :param i_tr: number of curren trajecotry
        :return:
        """
        traj_folder = self.save_dir + '/traj{}'.format(i_tr)
        self.joint_data_file = traj_folder + '/joint_angles_traj{}'.format(i_tr)
        self.image_folder = traj_folder + '/images'
        self.depth_image_folder = traj_folder + '/depth_images'
        if not os.path.exists(traj_folder):
            os.makedirs(traj_folder)
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
        if not os.path.exists(self.depth_image_folder):
            os.makedirs(self.depth_image_folder)

    def delete(self, i_tr):
        traj_folder = self.save_dir + '/traj{}'.format(i_tr)
        shutil.rmtree(traj_folder)
        print 'deleted {}'.format(traj_folder)

    def save(self, event= None, i_tr = None):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """

        #saving image
        if self.latest_image != None:
            print 'saving image'
            image_name =  self.image_folder+"/im{0}_time{1}.jpg".format(i_tr, self.time_stamp_im)
            cv2.imwrite(image_name, self.latest_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

        if self.latest_depth_image_raw!= None:
            d_image_name = self.depth_image_folder + "/depth_im{0}_time{1}.jpg".format(i_tr, self.time_stamp_d_im)
            cv2.imwrite(d_image_name, self.latest_depth_image_raw, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    def clean_shutdown(self):
        print 'TODO: clean shutdown!!'

if __name__ ==  '__main__':
    rec = KinectRecorder('/home/guser/Documents/sawyer_data/testrecording')


