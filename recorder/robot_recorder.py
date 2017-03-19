#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import intera_interface
import os
import shutil

from intera_interface import CHECK_VERSION

import thread

class MainRobotRecorder(object):
    def __init__(self, save_dir, rate=50, start_loop=True):
        """
        Records joint data to a file at a specified rate.
        rate: recording frequency in Hertz
        """
        side = "right"
        self._do_recording = False
        self._gripper = None
        self.gripper_name = '_'.join([side, 'gripper'])
        self._limb_right = intera_interface.Limb(side)

        self.save_dir = save_dir

        self.start_loop = start_loop  # whether to create a loop for saving or not
        if start_loop:  #for the simple setup in test_sdk.py
            self.joint_data_file = save_dir + '/test.csv'
            self.init_file()

            self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / rate), self.save)

        self.latest_image = None
        self.time_stamp_im = None  #timestamp of image

        # try:
        #     self._gripper = intera_interface.Gripper(side)
        #     rospy.loginfo("Electric gripper detected.")
        # except Exception as e:
        #     self._gripper = None
        #     rospy.loginfo("No electric gripper detected.")

        # # Verify Gripper Have No Errors and are Calibrated
        # if self._gripper:
        #     if self._gripper.has_error():
        #         self._gripper.reboot()                
        #     if not self._gripper.is_calibrated():
        #         self._gripper.calibrate()

        self._cuff = intera_interface.Cuff(side)


        rospy.Subscriber("/kinect2/hd/image_color", Image, self.store_latest_im)
        self.bridge = CvBridge()
        print "Recorder intialized."

        def spin_thread():
            print "started spin thread"
            rospy.spin()

        thread.start_new(spin_thread, ())



    def store_latest_im(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.latest_image = cv_image
        self.time_stamp_im = self._time_stamp()

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)


    def _time_stamp(self):
        return rospy.get_time()

    def stop(self):
        """
        Stop recording.
        """
        self._do_recording = False

    def start(self):
        """
        Stop recording.
        """
        self._do_recording = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._do_recording

    def init_file(self):
        joints_right = self._limb_right.joint_names()
        with open(self.joint_data_file, 'w+') as f:
            f.write('time,')
            temp_str = '' if self._gripper else '\n'
            f.write(','.join([j for j in joints_right]) + ',' + temp_str)

    def init_traj(self, i_tr):
        """
        :param i_tr: number of curren trajecotry
        :return:
        """
        traj_folder = self.save_dir + '/traj{}'.format(i_tr)
        self.joint_data_file = traj_folder + '/joint_angles_traj{}'.format(i_tr)
        self.image_folder = traj_folder + '/images'
        if not os.path.exists(traj_folder):
            os.makedirs(traj_folder)
            os.makedirs(self.image_folder)

        joints_right = self._limb_right.joint_names()
        with open(self.joint_data_file, 'w+') as f:
            f.write('time,')
            temp_str = '' if self._gripper else '\n'
            f.write(','.join([j for j in joints_right]) + ',' + temp_str)

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

        if not self._do_recording and self.start_loop:
            pass

        else:
            joints_right = self._limb_right.joint_names()
            with open(self.joint_data_file, 'a') as f:
                temp_str = '' if self._gripper else '\n'
                angles_right = [self._limb_right.joint_angle(j)
                                for j in joints_right]
                f.write("%f," % (self._time_stamp(),))
                f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)

            #saving image
            if self.latest_image != None:
                print 'saving image'
                image_name =  self.image_folder+"/im{0}_time{1}.jpg".format(i_tr, self.time_stamp_im)
                cv2.imwrite(image_name, self.latest_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])