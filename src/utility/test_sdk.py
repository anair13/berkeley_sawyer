#!/usr/bin/env python

import rospy

import src.inverse_kinematics
import src.robot_controller
from src.recorder import robot_recorder


def main1():
    """Test recording and moving joints by small amounts from keyboard input"""
    c = src.robot_controller.RobotController()
    r = robot_recorder.RobotRecorder("/media/guser/data/recordings/test.csv", 10)
    c.set_neutral()
    for i in range(5):
        n = input()
        j = c.joint_names[n]
        c.set_joint_delta(j, 0.1)

def main2():
    """Test inverse kinematics positions"""
    c = src.robot_controller.RobotController()
    r = robot_recorder.RobotRecorder(save_dir="/home/guser/sawyer_data/test_recording", rate=10)

    # for d in range(16, 40):
        # D = d / 100.0
    # start_pose = inverse_kinematics.get_pose_stamped(0.45, 0.16, 0.21, inverse_kinematics.EXAMPLE_O)
    # start_cmd = inverse_kinematics.get_joint_angles(start_pose)
    start_joints = c.limb.joint_angles()
    print start_joints

    desired_pose = src.inverse_kinematics.get_pose_stamped(0.6, -.2, 0.3, src.inverse_kinematics.EXAMPLE_O)
    cmd = src.inverse_kinematics.get_joint_angles(desired_pose, seed_cmd=start_joints)
    c.limb.set_joint_position_speed(.1)
    c.set_joints(cmd)

    c.set_gripper('close')
    # import pdb; pdb.set_trace()

if __name__ == '__main__':
    main2()

    rospy.spin()
