#!/usr/bin/env python
import numpy as np
from datetime import datetime
import pdb
import rospy
from intera_core_msgs.srv import (
    SolvePositionFK,
    SolvePositionFKRequest,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import inverse_kinematics
import robot_controller
from recorder import robot_recorder
import os
import cPickle


class Traj_aborted_except(Exception):
    pass

class Primitive_Executor(object):
    def __init__(self):

        self.num_traj = 50000

        # must be an uneven number
        self.state_sequence_length = 30 # number of snapshots that are taken


        self.ctrl = robot_controller.RobotController()
        self.recorder = robot_recorder.RobotRecorder(save_dir="/home/guser/sawyer_data/testrecording",
                                                     start_loop=False,
                                                     seq_len = self.state_sequence_length)

        self.alive_publisher = rospy.Publisher('still_alive', String, queue_size=10)
        # drive to neutral position:
        self.ctrl.set_neutral()

        limb = 'right'
        self.name_of_service = "ExternalTools/" + limb + "/PositionKinematicsNode/FKService"
        self.fksvc = rospy.ServiceProxy(self.name_of_service, SolvePositionFK)

        self.topen, self.t_down = None, None
        self.robot_move = True
        self.checkpoint_file = os.path.join(self.recorder.save_dir, 'checkpoint.txt')

        self.run_data_collection()

    def run_data_collection(self):

        # check if there is a checkpoint from which to resume
        if os.path.isfile(self.checkpoint_file):
            start_tr, start_grp = self.parse_ckpt()
            print 'resuming data collection at trajectory {} in group {}'.format(start_tr, start_grp)
            self.recorder.igrp = start_grp
            try:
                self.recorder.delete_traj(start_tr)
                self.recorder.delete_traj(start_tr+1)
            except:
                print 'trajectory was not deleted'
        else:
            start_tr = 0

        for tr in range(start_tr, self.num_traj):

            self.alive_publisher.publish('still alive!')

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

            self.write_ckpt(tr, self.recorder.igrp)

        self.ctrl.set_neutral()

    def write_ckpt(self, tr, i_grp):
        with open(self.checkpoint_file, 'w') as f:
            f.write("last trajectory, current group%f \n")
            f.write("{} {} \n".format(tr, i_grp))

    def parse_ckpt(self):
        with open(self.checkpoint_file, 'r') as f:
            i = 0
            for line in f:
                if i ==1:
                    numbers_str = line.split()
                    itr, igrp = [int(x) for x in numbers_str]  # map(float,numbers_str) works t
                i += 1
            return itr, igrp


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

        start_time = rospy.get_time()  # in seconds
        finish_time = start_time + duration  # in seconds
        print 'start time', start_time
        print 'finish_time', finish_time

        tsave = np.linspace(0, duration, self.state_sequence_length)
        print 'save times', tsave
        tact = tsave[::2]
        print 'cmd new pos times ', tact

        i_act = 0  # index of current commanded point
        i_save = 0  # index of current saved step

        self.ctrl.limb.set_joint_position_speed(.20)

        while rospy.get_time() < finish_time:
            self.curr_delta_time = rospy.get_time() - start_time

            if i_act < len(tact):
                if self.curr_delta_time > tact[i_act]:
                    # print 'current position error', self.des_pos - self.get_endeffector_pos(pos_only=True)
                    action_vec, des_joint_angles = self.act(i_act)  # after completing trajectory save final state
                    i_act += 1

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


            if self.curr_delta_time > tsave[i_save]:

                print 'saving index{}'.format(i_save)
                print 'action vec', action_vec
                self.recorder.save(i_save, action_vec, self.get_endeffector_pos())
                i_save += 1

        #saving the final state:
        self.recorder.save(i_save, action_vec, self.get_endeffector_pos())

        # stact = cPickle.load(open(self.recorder.state_action_pkl_file, 'r'))
        # print stact['jointangles']
        # print stact['actions']
        # print stact['endeffector_pos']
        # pdb.set_trace()

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
                self.ctrl.limb.move_to_joint_positions(des_joint_angles)
        except OSError:
            rospy.logerr('collision detected, stopping trajectory, going to reset robot...')
            rospy.sleep(.5)
            raise Traj_aborted_except('raising Traj_aborted_except')
        if self.ctrl.limb.has_collided():
            rospy.logerr('collision detected!!!')
            rospy.sleep(.5)
            raise Traj_aborted_except('raising Traj_aborted_except')

    def act(self, i_act):

        # sample action type:
        noptions = 3

        action = np.random.choice(range(noptions), p=[0.70, 0.2, 0.1])

        print 'action: ', i_act
        if action == 0:
            maxshift = .1
            posshift = np.concatenate((np.random.uniform(-maxshift, maxshift, 2), np.zeros(1)))
            self.des_pos += posshift
            self.des_pos = self.truncate_pos(self.des_pos)  # make sure not outside defined region
            print 'move to ', self.des_pos
        else:
            posshift = np.zeros(2)

        if action == 1:  # close gripper and hold for n steps
            close_cmd = np.random.randint(3, 5)
            print 'close and hold for ', close_cmd
            close_cmd = close_cmd
            self.topen = i_act + close_cmd
            self.ctrl.gripper.close()
            self.gripper_closed = True
        else:
            close_cmd = 0

        if action == 2:  # go up for n steps
            up_cmd = np.random.randint(3, 5)
            self.t_down = i_act + up_cmd
            delta_up = .1
            self.des_pos[2] = self.lower_height + delta_up
            self.gripper_up = True
            print 'stay up for ', up_cmd
        else:
            up_cmd = 0

        if self.gripper_closed:
            if i_act == self.topen:
                self.ctrl.gripper.open()
                print 'opening gripper'
                self.gripper_closed = False

        if self.gripper_up:
            if i_act == self.t_down:
                self.des_pos[2] = self.lower_height
                print 'going down'
                self.gripper_up = False


        action_vec = np.array([
                               posshift[0],
                               posshift[1],
                               close_cmd,
                               up_cmd
                               ])
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

        return action_vec, des_joint_angles

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

def main():
    pexec = Primitive_Executor()


if __name__ == '__main__':
    main()
