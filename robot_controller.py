#!/usr/bin/env python

import argparse

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

class RobotController(object):

    def __init__(self):
        """Initializes a controller for the robot"""

        print("Initializing node... ")
        rospy.init_node("sawyer_custom_controller")
        rospy.on_shutdown(self.clean_shutdown)
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled

        self.limb = intera_interface.Limb("right")
    
        try:
            self.gripper = intera_interface.Gripper("right")
        except:
            self.has_gripper = False
            rospy.logerr("Could not initalize the gripper.")
        else:
            self.has_gripper = True

        self.joints = self.limb.joint_names()

    def set_joint_delta(self, joint_name, delta):
        current_position = self.limb.joint_angle(joint_name)
        self.set_joint(joint_name, current_position + delta)

    def set_joint(self, joint_name, pos):
        joint_command = {joint_name: pos}
        self.limb.set_joint_positions(joint_command)

    def set_gripper(action):
        if has_gripper:
            if action == "close":
                gripper.close()
            elif action == "open":
                gripper.open()
            elif action == "calibrate":
                gripper.calibrate()

    def set_neutral(self):
        self.limb.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example.")
        # if not init_state:
        #     print("Disabling robot...")
            # rs.disable()