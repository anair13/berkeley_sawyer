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

import inverse_kinematics
import robot_controller
from recorder import robot_recorder


class Traj_aborted_except(Exception):
    pass

class Primitive_Executor(object):
    def __init__(self):

        self.num_traj = 500

        self.state_sequence_length = 30 # number of snapshots that are taken
        self.num_act = self.state_sequence_length / 2 - 1  # number of different positions on trajectory

        self.ctrl = robot_controller.RobotController()
        self.recorder = robot_recorder.RobotRecorder(save_dir="/home/guser/sawyer_data/test_recording",
                                                     start_loop=False,
                                                     seq_len = self.state_sequence_length)

        # drive to neutral position:
        self.ctrl.set_neutral()
        import pdb; pdb.set_trace()

        limb = 'right'
        self.name_of_service = "ExternalTools/" + limb + "/PositionKinematicsNode/FKService"
        self.fksvc = rospy.ServiceProxy(self.name_of_service, SolvePositionFK)

        self.close_nstep, self.tclose, self.up_nstep, self.t_up = None, None, None, None
        self.robot_move = True
        self.run_data_collection()

    def run_data_collection(self):

        for tr in range(self.num_traj):

            tstart = datetime.now()
            # self.run_trajectory_const_speed(tr)
            done = False
            while not done:
                try:
                    self.run_trajectory(tr)
                    done = True
                except Traj_aborted_except:
                    self.recorder.delete(tr)

            delta = datetime.now() - tstart
            print 'trajectory {0} took {1} seconds'.format(tr, delta.total_seconds())


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
        self.gripper_up = True
        self.recorder.init_traj(i_tr)

        self.lower_height = 0.22
        self.xlim = [0.44, 0.83]  # min, max in cartesian X-direction
        self.ylim = [-0.27, 0.18]  # min, max in cartesian Y-direction
        self.des_pos = np.array([np.random.uniform(self.xlim[0], self.xlim[1]),
                        np.random.uniform(self.ylim[0], self.ylim[1])])


        self.close_nstep, self.tclose, self.up_nstep, self.t_up = 0, 0, 0, 0
        self.next_wypt = np.array([self.des_pos[0], self.des_pos[1], self.lower_height])

        duration = 10  # duration of trajectory in seconds

        start_time = rospy.get_time()  # in seconds
        finish_time = start_time + duration  # in seconds
        print 'start time', start_time
        print 'finish_time', finish_time

        tsave = np.linspace(0, duration, self.state_sequence_length)
        print 'save times', tsave
        tact = np.linspace(0, duration,
                           self.num_act + 1)  # +1 because the first time is when the endeffecotr is at the initial position
        print 'cmd new pos times ', tact

        i_act = 0  # index of current commanded point
        i_save = 0  # index of current saved step

        self.ctrl.limb.set_joint_position_speed(.15)

        while rospy.get_time() < finish_time:
            self.curr_delta_time = rospy.get_time() - start_time

            if self.curr_delta_time > tact[i_act]:
                action_vec, des_joint_angles, des_cartpos = self.act(i_act)  # after completing trajectory save final state
                i_act += 1

            try:
                if self.robot_move:
                    self.ctrl.limb.set_joint_positions(des_joint_angles)
            except OSError:
                rospy.logerr('collision detected, stopping trajectory, going to reset robot...')
                rospy.sleep(.5)
                raise Traj_aborted_except('raising Traj_aborted_except')
            if self.ctrl.limb.has_collided():
                rospy.logerr('collision detected!!!')
                rospy.sleep(.5)
                raise Traj_aborted_except('raising Traj_aborted_except')

            # print 'current position error', des_cartpos - self.get_endeffector_pos(pos_only=True)
            if self.curr_delta_time > tsave[i_save]:
                print 'saving index{}'.format(i_save)
                self.recorder.save(i_save, action_vec)
                i_save += 1

        action_vec = np.zeros_like(action_vec)
        #saving the final state:
        self.recorder.save(i_save, action_vec)

    def act(self, i_act):

        # sample action type:
        noptions = 3
        action = np.random.choice(range(noptions), p=[0.80, 0.1, 0.1])
        print 'action: ', i_act
        if action == 0:
            maxshift = .1
            posshift = np.random.uniform(-maxshift, maxshift, 2)
            self.des_pos += posshift
            self.des_pos = self.truncate_pos(self.des_pos)  # make sure not outside defined region
            self.next_wypt[0] = self.des_pos[0]
            self.next_wypt[1] = self.des_pos[1]
            print 'move to ', self.next_wypt
        else:
            posshift = np.zeros(2)
        if action == 1:  # close gripper and hold for n steps
            self.close_nstep = np.random.randint(1, 6)
            print 'close and hold for ', self.close_nstep
            self.tclose = i_act
            self.ctrl.gripper.close()
            self.gripper_closed = True
        else:
            close_nstep = 0
        if action == 2:  # go up for n steps
            self.up_nstep = np.random.randint(1, 6)
            self.t_up = i_act
            delta_up = .1
            self.next_wypt[2] = self.lower_height + delta_up
            self.gripper_up = True
            print 'stay up for ', self.up_nstep
        else:
            up_nstep = 0
        if self.gripper_closed:
            if i_act > (self.tclose + self.close_nstep):
                self.ctrl.gripper.open()
                print 'opening gripper'
            self.gripper_closed = False
        if self.gripper_up:
            if i_act > (self.t_up + self.up_nstep):
                self.next_wypt[2] = self.lower_height
                print 'going down'
                self.gripper_up = False
        action_vec = np.array([1 if action == 0 else 0,  # move
                               posshift[0],
                               posshift[1],
                               1 if action == 1 else 0,  # close
                               self.close_nstep,
                               1 if action == 2 else 0,  # up nstep
                               self.up_nstep
                               ])
        desired_pose = inverse_kinematics.get_pose_stamped(self.next_wypt[0],
                                                           self.next_wypt[1],
                                                           self.next_wypt[2],
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

        return action_vec, des_joint_angles, self.next_wypt

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


def main():
    pexec = Primitive_Executor()


if __name__ == '__main__':
    main()
