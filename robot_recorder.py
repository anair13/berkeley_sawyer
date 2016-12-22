#!/usr/bin/env python

import rospy

import intera_interface

from intera_interface import CHECK_VERSION


class RobotRecorder(object):
    def __init__(self, filename, rate=50):
        """
        Records joint data to a file at a specified rate.
        """
        side = "right"
        self.gripper_name = '_'.join([side, 'gripper'])
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / rate), self.save)
        self._done = False

        self._limb_right = intera_interface.Limb(side)
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
        self._gripper = None

        self._cuff = intera_interface.Cuff(side)

        self._filename = filename
        self.init_file()

    def _time_stamp(self):
        return rospy.get_time()

    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def init_file(self):
        joints_right = self._limb_right.joint_names()
        with open(self._filename, 'w+') as f:
            f.write('time,')
            temp_str = '' if self._gripper else '\n'
            f.write(','.join([j for j in joints_right]) + ',' + temp_str)

    def save(self, event):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """
        joints_right = self._limb_right.joint_names()
        with open(self._filename, 'a') as f:
            temp_str = '' if self._gripper else '\n'
            angles_right = [self._limb_right.joint_angle(j)
                            for j in joints_right]
            f.write("%f," % (self._time_stamp(),))
            f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)