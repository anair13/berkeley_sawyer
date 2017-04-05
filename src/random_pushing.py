#!/usr/bin/env python
import numpy as np
from datetime import datetime

import rospy
from intera_core_msgs.srv import (
    SolvePositionFK,
    SolvePositionFKRequest,
)
from sensor_msgs.msg import JointState

import inverse_kinematics
import robot_controller
from recorder import robot_recorder


class RandomPusher(object):
    def __init__(self):
        """Test inverse kinematics positions"""
        self.ctrl = robot_controller.RobotController()
        self.recorder = robot_recorder.RobotRecorder(save_dir="/home/guser/sawyer_data/test_recording", start_loop=False)

        # drive to neutral position:
        self.ctrl.set_neutral()
        # import pdb; pdb.set_trace()

        self.num_traj = 10


        limb = 'right'
        self.name_of_service = "ExternalTools/" + limb + "/PositionKinematicsNode/FKService"
        self.fksvc = rospy.ServiceProxy(self.name_of_service, SolvePositionFK)

        self.run_data_collection()

    def run_data_collection(self):

        for tr in range(self.num_traj):

            tstart = datetime.now()
            # self.run_trajectory_const_speed(tr)
            done = False
            while not done:
                try:
                    self.run_trajectory_smooth(tr)
                    done = True
                except ValueError:
                    self.recorder._delete_traj_local(tr)

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



    def run_trajectory_smooth(self, i_tr):

        self.ctrl.set_neutral(speed= 0.4)
        self.recorder.init_traj(i_tr)

        num_pos = 15  # number of different positions on trajectory

        z = 0.22

        xlim = [0.44, 0.74]  # min, max in cartesian X-direction
        ylim = [-0.27, 0.2]  # min, max in cartesian Y-direction
        goalpoints = []

        maxshift = .1
        pos = np.array([np.random.uniform(xlim[0], xlim[1]),
                        np.random.uniform(ylim[0], ylim[1])])
        for itr in range(num_pos-1):
            goalpoints.append(np.array([pos[0], pos[1],z]))
            pos += np.random.uniform(-maxshift, maxshift,2)

            if pos[0] > xlim[1]:
                pos[0] = xlim[1]
            if pos[0] < xlim[0]:
                pos[0] = xlim[0]
            if pos[1] > ylim[1]:
                pos[1] = ylim[1]
            if pos[1] < ylim[0]:
                pos[1] = ylim[0]

        # print "goalpoints:"
        # print goalpoints

        self.ctrl.limb.set_joint_position_speed(.2)

        for itr, curr_waypt in enumerate(goalpoints):

            start_joints = self.ctrl.limb.joint_angles()
            desired_pose = inverse_kinematics.get_pose_stamped(curr_waypt[0],
                                                               curr_waypt[1],
                                                               curr_waypt[2],
                                                               inverse_kinematics.EXAMPLE_O)
            try:
                cmd = inverse_kinematics.get_joint_angles(desired_pose, seed_cmd=start_joints,
                                                          use_advanced_options=True)
            except ValueError:
                rospy.logerr('no inverser kinematics solution found, going to reset robot...')
                current_joints = self.ctrl.limb.joint_angles()
                self.ctrl.limb.move_to_joint_positions(current_joints)
                raise ValueError

            try:
                self.ctrl.limb.move_to_joint_positions(cmd, timeout=2.)
            except OSError:
                rospy.logerr('collision detected, stopping trajectory, going to reset robot...')
                rospy.sleep(.5)
                raise ValueError

            if self.ctrl.limb.has_collided():
                rospy.logerr('collision detected!!!')
                rospy.sleep(.5)
                raise ValueError

            self.recorder.save(i_tr=itr)


    def run_trajectory_const_speed(self, i_tr):

        self.ctrl.set_neutral()
        self.recorder.init_traj(i_tr)

        num_save = 15  #number of snapshots on trajecotry
        num_pos = 4 # number of different positions on trajectory
        duration = 4 # duration of trajectory in seconds

        z = 0.21

        xlim = [0.43, 0.74]  # min, max in cartesian X-direction
        ylim = [-0.27, 0.2]  # min, max in cartesian Y-direction
        goalpoints = []
        goalpoints.append(self.get_endeffector_pos(pos_only=True))
        for i in range(num_pos):
            goalpoints.append(np.array([np.random.uniform(xlim[0], xlim[1]),
                                        np.random.uniform(ylim[0], ylim[1]), z]))

        print "goalpoints:"
        print goalpoints

        # import pdb;
        # pdb.set_trace()

        start_time = rospy.get_time()  # in seconds
        finish_time = start_time + duration  # in seconds
        print 'start time', start_time
        print 'finish_time', finish_time

        tsave = np.linspace(0, duration, num_save)
        print 'save times', tsave
        tpos = np.linspace(0, duration, num_pos +1)   # +1 because the first time is when the endeffecotr is at the initial position
        print 'cmd new pos times ', tpos


        goto_pt = 0  # index of current commanded point
        idx_save = 0 # index of current saved step


        self.ctrl.limb.set_joint_position_speed(1)

        while rospy.get_time() < finish_time:
            self.curr_delta_time = rospy.get_time() - start_time

            if self.curr_delta_time > tpos[goto_pt]:
                goto_pt += 1

                prev_goalpoint = goalpoints[goto_pt-1]
                next_goalpoint = goalpoints[goto_pt]
                tprev = tpos[goto_pt-1]
                tnext = tpos[goto_pt]
                print '-------------------------------------------------------'
                print 'going to point number {0}:'.format(goto_pt)
                print 'tprev', tprev
                print 'tnext', tnext
                print 'prev_goalpoint', prev_goalpoint
                print 'next_goalpoint', next_goalpoint
                print 'has_collided: ', self.ctrl.limb.has_collided()


            newpos = self.interpolate(prev_goalpoint, next_goalpoint, tprev, tnext)

            start_joints = self.ctrl.limb.joint_angles()
            desired_pose = inverse_kinematics.get_pose_stamped(newpos[0], newpos[1], newpos[2], inverse_kinematics.EXAMPLE_O)
            try:
                cmd = inverse_kinematics.get_joint_angles(desired_pose, seed_cmd=start_joints, use_advanced_options=True)
            except ValueError:
                print 'stopping trajectory, going to reset robot...'
                current_joints = self.ctrl.limb.joint_angles()
                self.ctrl.limb.set_joint_positions(current_joints)
                import pdb; pdb.set_trace()
                return

            # print 'joint pos cmd:'
            # print cmd
            # rospy.sleep(.01)

            self.ctrl.limb.set_joint_positions(cmd)
            # print 'current position error', newpos - self.get_endeffector_pos(pos_only=True)

            # if rospy.get_time()  > tsave[idx_save]:
            #     print 'saving index{}'.format(idx_save)
            #     idx_save += 1
            #     self.recorder.save(i_tr= idx_save)

    def interpolate(self, previous_goalpoint, next_goalpoint, t_prev, t_next):
        """
        interpolate cartesian positions (x,y,z) between last goalpoint and previous goalpoint at the current time
        :param previous_goalpoint:
        :param next_goalpoint:
        :param goto_point:
        :param tnewpos:
        :return:
        """
        des_pos = previous_goalpoint + (next_goalpoint - previous_goalpoint) * (self.curr_delta_time- t_prev)/ (t_next - t_prev)

        # print 'current_delta_time: ', self.curr_delta_time
        # print "interpolated pos:", des_pos

        return des_pos


def main():
    pusher = RandomPusher()
    pusher.run_data_collection()


if __name__ == '__main__':
    main()
