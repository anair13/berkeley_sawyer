#!/usr/bin/env python
import numpy as np
from datetime import datetime
import pdb
import rospy

import socket
if socket.gethostname() == 'kullback':
    from intera_core_msgs.srv import (
        SolvePositionFK,
        SolvePositionFKRequest,
    )

from sensor_msgs.msg import JointState
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError

import inverse_kinematics
import robot_controller
from recorder import robot_recorder
import os
import cPickle
from std_msgs.msg import Float32
from std_msgs.msg import Int64

from berkeley_sawyer.srv import *


class Traj_aborted_except(Exception):
    pass

class Visual_MPC_Client():
    def __init__(self):
        self.num_traj = 50000

        # must be an uneven number
        self.action_sequence_length = 25 # number of snapshots that are taken

        self.ctrl = robot_controller.RobotController()
        self.recorder = robot_recorder.RobotRecorder(save_dir="/home/guser/sawyer_data/visual_mpc",
                                                     start_loop=False,
                                                     seq_len = self.action_sequence_length)
        # drive to neutral position:
        ################# self.ctrl.set_neutral()
        self.get_action_func = rospy.ServiceProxy('get_action', get_action)
        self.robot_move = True

        self.imp_ctrl_publisher = rospy.Publisher('desired_joint_pos', JointState, queue_size=1)
        self.imp_ctrl_release_spring_pub = rospy.Publisher('release_spring', Float32, queue_size=10)
        self.imp_ctrl_active = rospy.Publisher('imp_ctrl_active', Int64, queue_size=10)
        self.name_of_service = "ExternalTools/right/PositionKinematicsNode/FKService"
        self.fksvc = rospy.ServiceProxy(self.name_of_service, SolvePositionFK)

        # start!
        self.images = None

        self.use_imp_ctrl = True
        self.robot_move = False
        self.save_active = False

        self.bridge = CvBridge()

        rospy.sleep(1)
        self.run_visual_mpc()


    def collect_goal_image(self):
        goalimage_folder = os.path.dirname(os.path.realpath(__file__)) + '/goalimages'

    def imp_ctrl_release_spring(self, maxstiff):
        self.imp_ctrl_release_spring_pub.publish(maxstiff)


    def run_visual_mpc(self):

        # check if there is a checkpoint from which to resume
        start_tr = 0

        for tr in range(start_tr, self.num_traj):


            tstart = datetime.now()
            # self.run_trajectory_const_speed(tr)
            done = False
            while not done:
                try:
                    self.run_trajectory(tr)
                    done = True
                except Traj_aborted_except:
                    self.recorder.delete_traj(tr)

            delta = datetime.now() - tstart
            print 'trajectory {0} took {1} seconds'.format(tr, delta.total_seconds())

            if (tr% 30) == 0 and tr!= 0:
                self.redistribute_objects()

        self.ctrl.set_neutral()


    def get_endeffector_pos(self, pos_only=True):
        """
        :param pos_only: only return postion
        :return:
        """

        fkreq = SolvePositionFKRequest()
        joints = JointState()
        joints.name = self.ctrl.limb.joint_names()
        joints.position = [self.ctrl.limb.joint_angle(j)
                        for j in joints.name]

        # Add desired pose for forward kinematics
        fkreq.configuration.append(joints)
        fkreq.tip_names.append('right_hand')
        try:
            rospy.wait_for_service(self.name_of_service, 5)
            resp = self.fksvc(fkreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        pos = np.array([resp.pose_stamp[0].pose.position.x,
                         resp.pose_stamp[0].pose.position.y,
                         resp.pose_stamp[0].pose.position.z])
        return pos

    def run_trajectory(self, i_tr):

        self.ctrl.set_neutral(speed= 0.3)
        self.ctrl.gripper.open()

        self.gripper_closed = False
        self.gripper_up = False
        self.recorder.init_traj(i_tr)

        self.lower_height = 0.21
        self.xlim = [0.44, 0.83]  # min, max in cartesian X-direction
        self.ylim = [-0.27, 0.18]  # min, max in cartesian Y-direction

        startpos = np.array([np.random.uniform(self.xlim[0], self.xlim[1]), np.random.uniform(self.ylim[0], self.ylim[1])])
        self.des_pos = np.concatenate([startpos, np.asarray([self.lower_height])], axis=0)

        self.topen, self.t_down = 0, 0

        duration = 10  # duration of trajectory in seconds

        #move to start:
        self.move_to_startpos()

        i_save = 0  # index of current saved step
        for i_act in range(self.action_sequence_length):

            action_vec = self.query_action()
            self.apply_act(action_vec, i_act)

            if self.save_active:
                self.recorder.save(i_save, action_vec, self.get_endeffector_pos())
                i_save += 1


    def query_action(self):
        try:
            rospy.wait_for_service('get_kinectdata', 10)
            imagemain = self.bridge.cv2_to_imgmsg(self.recorder.ltob.img_cropped)
            imageaux1 = self.recorder.ltob_aux1.img_cropped
            state = self.get_endeffector_pos()
            action_vec = self.get_action_func(imagemain, imageaux1, list(state))

        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

        return action_vec


    def move_with_impedance(self, des_joint_angles):
        """
        non-blocking
        """
        js = JointState()
        js.name = self.ctrl.limb.joint_names()
        js.position = [des_joint_angles[n] for n in js.name]
        self.imp_ctrl_publisher.publish(js)

    def move_with_impedance_sec(self, cmd, tsec = 2.):
        """
        blocking
        """
        tstart = rospy.get_time()
        delta_t = 0
        while delta_t < tsec:
            delta_t = rospy.get_time() - tstart
            self.move_with_impedance(cmd)

    def set_neutral_with_impedance(self):
        neutral_jointangles = [0.412271, -0.434908, -1.198768, 1.795462, 1.160788, 1.107675, 2.068076]
        cmd = dict(zip(self.ctrl.limb.joint_names(), neutral_jointangles))
        self.imp_ctrl_release_spring(50)
        self.move_with_impedance_sec(cmd)

    def move_to_startpos(self):
        desired_pose = inverse_kinematics.get_pose_stamped(self.des_pos[0],
                                                           self.des_pos[1],
                                                           self.des_pos[2],
                                                           inverse_kinematics.EXAMPLE_O)
        start_joints = self.ctrl.limb.joint_angles()
        try:
            des_joint_angles = inverse_kinematics.get_joint_angles(desired_pose, seed_cmd=start_joints,
                                                                   use_advanced_options=True)
        except ValueError:
            rospy.logerr('no inverse kinematics solution found, '
                         'going to reset robot...')
            current_joints = self.ctrl.limb.joint_angles()
            self.ctrl.limb.set_joint_positions(current_joints)
            raise Traj_aborted_except('raising Traj_aborted_except')
        try:
            if self.robot_move:
                if self.use_imp_ctrl:
                    self.imp_ctrl_release_spring(30)
                    self.move_with_impedance_sec(des_joint_angles)
                else:
                    self.ctrl.limb.move_to_joint_positions(des_joint_angles)
        except OSError:
            rospy.logerr('collision detected, stopping trajectory, going to reset robot...')
            rospy.sleep(.5)
            raise Traj_aborted_except('raising Traj_aborted_except')
        if self.ctrl.limb.has_collided():
            rospy.logerr('collision detected!!!')
            rospy.sleep(.5)
            raise Traj_aborted_except('raising Traj_aborted_except')

    def apply_act(self, action_vec, i_act):

        self.des_pos += action_vec[0:2]
        self.des_pos = self.truncate_pos(self.des_pos)  # make sure not outside defined region

        self.imp_ctrl_release_spring(120.)
        close_cmd = action_vec[2]
        if close_cmd != 0:
            self.topen = i_act + close_cmd
            self.ctrl.gripper.close()
            self.gripper_closed = True

        up_cmd = action_vec[3]
        delta_up = .1
        if up_cmd != 0:
            self.t_down = i_act + up_cmd
            self.des_pos[2] = self.lower_height + delta_up
            self.gripper_up = True

        if self.gripper_closed:
            if i_act == self.topen:
                self.ctrl.gripper.open()
                print 'opening gripper'
                self.gripper_closed = False

        if self.gripper_up:
            if i_act == self.t_down:
                self.des_pos[2] = self.lower_height
                print 'going down'
                self.imp_ctrl_release_spring(30.)
                self.gripper_up = False

        desired_pose = inverse_kinematics.get_pose_stamped(self.des_pos[0],
                                                           self.des_pos[1],
                                                           self.des_pos[2],
                                                           inverse_kinematics.EXAMPLE_O)
        start_joints = self.ctrl.limb.joint_angles()
        try:
            des_joint_angles = inverse_kinematics.get_joint_angles(desired_pose, seed_cmd=start_joints,
                                                                   use_advanced_options=True)
        except ValueError:
            rospy.logerr('no inverse kinematics solution found, '
                         'going to reset robot...')
            current_joints = self.ctrl.limb.joint_angles()
            self.ctrl.limb.set_joint_positions(current_joints)
            raise Traj_aborted_except('raising Traj_aborted_except')

        try:
            if self.robot_move:
                self.ctrl.limb.set_joint_positions(des_joint_angles)
                # print des_joint_angles
        except OSError:
            rospy.logerr('collision detected, stopping trajectory, going to reset robot...')
            rospy.sleep(.5)
            raise Traj_aborted_except('raising Traj_aborted_except')
        if self.ctrl.limb.has_collided():
            rospy.logerr('collision detected!!!')
            rospy.sleep(.5)
            raise Traj_aborted_except('raising Traj_aborted_except')


    def truncate_pos(self, pos):

        xlim = self.xlim
        ylim = self.ylim

        if pos[0] > xlim[1]:
            pos[0] = xlim[1]
        if pos[0] < xlim[0]:
            pos[0] = xlim[0]
        if pos[1] > ylim[1]:
            pos[1] = ylim[1]
        if pos[1] < ylim[0]:
            pos[1] = ylim[0]

        return  pos


    def redistribute_objects(self):
        """
        Loops playback of recorded joint position waypoints until program is
        exited
        """
        with open('/home/guser/catkin_ws/src/berkeley_sawyer/src/waypts.pkl', 'r') as f:
            waypoints = cPickle.load(f)
        rospy.loginfo("Waypoint Playback Started")

        # Set joint position speed ratio for execution
        self.ctrl.limb.set_joint_position_speed(.2)

        # Loop until program is exited
        do_repeat = True
        n_repeat = 0
        while do_repeat and (n_repeat < 2):
            do_repeat = False
            n_repeat += 1
            for i, waypoint in enumerate(waypoints):
                if rospy.is_shutdown():
                    break
                try:
                    print 'going to waypoint ', i
                    self.ctrl.limb.move_to_joint_positions(waypoint, timeout=5.0)
                except:
                    do_repeat = True
                    break



if __name__ == '__main__':
    mpc = Visual_MPC_Client()
