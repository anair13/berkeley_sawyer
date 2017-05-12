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

from std_msgs.msg import Float32
from std_msgs.msg import Int64

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
        seq_length = 16
        n_traj_per_run = 4
        self.act_every = 4
        self.duration = 12  # duration of trajectory in seconds
        self.state_sequence_length = seq_length*n_traj_per_run # number of snapshots that are taken

        self.ctrl = robot_controller.RobotController()
        self.recorder = robot_recorder.RobotRecorder(save_dir="/home/guser/sawyer_data/newrecording",
                                                     start_loop=False,
                                                     seq_len = self.state_sequence_length)

        self.alive_publisher = rospy.Publisher('still_alive', String, queue_size=10)


        self.imp_ctrl_publisher = rospy.Publisher('desired_joint_pos', JointState, queue_size=1)
        self.imp_ctrl_release_spring_pub = rospy.Publisher('release_spring', Float32, queue_size=10)
        self.imp_ctrl_active = rospy.Publisher('imp_ctrl_active', Int64, queue_size=10)

        rospy.sleep(1)
        # drive to neutral position:
        self.imp_ctrl_active.publish(0)
        self.ctrl.set_neutral()
        self.set_neutral_with_impedance()
        self.imp_ctrl_active.publish(1)
        rospy.sleep(1)

        limb = 'right'
        self.name_of_service = "ExternalTools/" + limb + "/PositionKinematicsNode/FKService"
        self.fksvc = rospy.ServiceProxy(self.name_of_service, SolvePositionFK)

        self.topen, self.t_down = None, None

        self.use_imp_ctrl = True
        self.robot_move = True
        self.save_active = True
        self.checkpoint_file = os.path.join(self.recorder.save_dir, 'checkpoint.txt')

        self.control_rate = rospy.Rate(1000)
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

        accum_time = 0
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

            if ((tr+1)% 20) == 0:
                self.redistribute_objects()

            delta = datetime.now() - tstart
            print 'trajectory {0} took {1} seconds'.format(tr, delta.total_seconds())
            accum_time += delta.total_seconds()

            avg_nstep = 80
            if ((tr+1)% avg_nstep) == 0:
                average = accum_time/avg_nstep
                self.write_timing_file(average)
                accum_time = 0

            self.write_ckpt(tr, self.recorder.igrp)
            self.alive_publisher.publish('still alive!')


    def write_ckpt(self, tr, i_grp):
        with open(self.checkpoint_file, 'w') as f:
            f.write("last trajectory, current group%f \n")
            f.write("{} {} \n".format(tr, i_grp))

    def write_timing_file(self, avg):
        with open(os.path.join(self.recorder.save_dir, 'timingfile.txt'), 'w') as f:
            f.write("average duration for trajectory (including amortization of redistribution trajectory): {} \n".format(avg))
            f.write("expected number of trajectory per day: {} \n".format(24.*3600./avg))


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

        # self.ctrl.set_neutral(speed= 0.3) ########################
        self.set_neutral_with_impedance()

        self.ctrl.gripper.open()
        self.gripper_closed = False
        self.gripper_up = False
        if self.save_active:
            self.recorder.init_traj(i_tr)

        self.lower_height = 0.20
        self.xlim = [0.44, 0.83]  # min, max in cartesian X-direction
        self.ylim = [-0.27, 0.18]  # min, max in cartesian Y-direction

        startpos = np.array([np.random.uniform(self.xlim[0], self.xlim[1]), np.random.uniform(self.ylim[0], self.ylim[1])])
        self.des_pos = np.concatenate([startpos, np.asarray([self.lower_height])], axis=0)

        self.topen, self.t_down = 0, 0



        #move to start:
        self.move_to_startpos()

        start_time = rospy.get_time()  # in seconds
        finish_time = start_time + self.duration  # in seconds
        print 'start time', start_time
        print 'finish_time', finish_time

        tsave = np.linspace(0, self.duration, self.state_sequence_length)
        print 'save times', tsave

        tact = tsave[::self.act_every]
        print 'cmd new pos times ', tact

        i_act = 0  # index of current commanded point
        i_save = 0  # index of current saved step

        self.ctrl.limb.set_joint_position_speed(.20)

        while rospy.get_time() < finish_time:
            self.curr_delta_time = rospy.get_time() - start_time

            if i_act < len(tact):
                if self.curr_delta_time > tact[i_act]:
                    print 'current position error', self.des_pos - self.get_endeffector_pos(pos_only=True)
                    action_vec, des_joint_angles = self.act_joint(i_act)  # after completing trajectory save final state
                    i_act += 1

            try:
                if self.robot_move:
                    if self.use_imp_ctrl:
                        self.move_with_impedance(des_joint_angles)
                    else:
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
                if self.save_active:
                    self.recorder.save(i_save, action_vec, self.get_endeffector_pos())
                i_save += 1


            self.control_rate.sleep()

        #saving the final state:
        if self.save_active:
            self.recorder.save(i_save, action_vec, self.get_endeffector_pos())

        if i_save != (self.state_sequence_length -1):
            raise Traj_aborted_except('trajectory not complete!')

        self.goup()

    def goup(self):
        print "going up at the end.."
        delta_up = .1
        self.des_pos[2] = self.lower_height + delta_up
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
        self.imp_ctrl_release_spring(150)
        self.move_with_impedance_sec(des_joint_angles, tsec=.5)

    def imp_ctrl_release_spring(self, maxstiff):
        self.imp_ctrl_release_spring_pub.publish(maxstiff)

    def move_with_impedance(self, des_joint_angles):
        """
        non-blocking
        """
        js = JointState()
        js.name = self.ctrl.limb.joint_names()
        js.position = [des_joint_angles[n] for n in js.name]
        self.imp_ctrl_publisher.publish(js)

    def move_with_impedance_sec(self, cmd, tsec = 1.5):
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
        self.imp_ctrl_release_spring(60)
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


    def act_joint(self, i_act):
        """
        only combined actions
        :param i_act:
        :return:
        """

        maxshift = .07
        posshift = np.concatenate((np.random.uniform(-maxshift, maxshift, 2), np.zeros(1)))
        self.des_pos += posshift
        self.des_pos = self.truncate_pos(self.des_pos)  # make sure not outside defined region


        self.imp_ctrl_release_spring(120.)
        close_cmd = np.random.choice(range(5), p=[0.8, 0.05, 0.05, 0.05, 0.05])
        if close_cmd != 0:
            self.topen = i_act + close_cmd
            self.ctrl.gripper.close()
            self.gripper_closed = True


        up_cmd = np.random.choice(range(5), p=[0.9, 0.025, 0.025, 0.025, 0.025])
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

    def act_distinct(self, i_act):
        """
        only perfom one action at a time
        :param i_act:
        :return:
        """

        # sample action type:
        noptions = 3

        action = np.random.choice(range(noptions), p=[0.70, 0.2, 0.1])

        print 'action: ', i_act
        if action == 0:
            maxshift = .07
            posshift = np.concatenate((np.random.uniform(-maxshift, maxshift, 2), np.zeros(1)))
            self.des_pos += posshift
            self.des_pos = self.truncate_pos(self.des_pos)  # make sure not outside defined region
            print 'move to ', self.des_pos

            self.imp_ctrl_release_spring(150.)
        else:
            posshift = np.zeros(2)

        if action == 1:  # close gripper and hold for n steps
            close_cmd = np.random.randint(3, 5)
            print 'close and hold for ', close_cmd
            self.topen = i_act + close_cmd
            self.ctrl.gripper.close()
            self.gripper_closed = True
            self.imp_ctrl_release_spring(150.)
        else:
            close_cmd = 0

        if action == 2:  # go up for n steps
            up_cmd = np.random.randint(3, 5)
            self.t_down = i_act + up_cmd
            delta_up = .1
            self.des_pos[2] = self.lower_height + delta_up
            self.gripper_up = True
            print 'stay up for ', up_cmd
            self.imp_ctrl_release_spring(150)
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
                self.imp_ctrl_release_spring(80.)
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
        # self.imp_ctrl_active.publish(0)
        while do_repeat and (n_repeat < 2):
            do_repeat = False
            n_repeat += 1
            for i, waypoint in enumerate(waypoints):
                if rospy.is_shutdown():
                    break
                try:
                    print 'going to waypoint ', i

                    if self.imp_ctrl_active:
                        if i < 4:
                            self.imp_ctrl_release_spring(30)
                        else:
                            self.imp_ctrl_release_spring(65)
                        self.move_with_impedance_sec(waypoint, tsec=1.0)
                    else:
                        self.ctrl.limb.move_to_joint_positions(waypoint, timeout=5.0)
                except:
                    do_repeat = True
                    break
        # self.imp_ctrl_active.publish(1)

def main():
    pexec = Primitive_Executor()


if __name__ == '__main__':
    main()
