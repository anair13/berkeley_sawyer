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


class Primitive_Executor(object):
    def __init__(self):
        """Test inverse kinematics positions"""
        self.ctrl = robot_controller.RobotController()
        self.recorder = robot_recorder.RobotRecorder(save_dir="/home/guser/sawyer_data/test_recording", start_loop=False)

        # drive to neutral position:
        self.ctrl.set_neutral()
        import pdb; pdb.set_trace()

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


    def run_trajectory_smooth(self, i_tr):

        self.ctrl.set_neutral(speed= 0.2)
        self.ctrl.gripper.open()
        gripper_closed = False
        gripper_up = True


        next_wypt = self.get_endeffector_pos()

        self.recorder.init_traj(i_tr)

        num_actions = 15  # number of different positions on trajectory

        lower_height = 0.22

        self.xlim = [0.44, 0.74]  # min, max in cartesian X-direction
        self.ylim = [-0.27, 0.2]  # min, max in cartesian Y-direction
        des_pos = np.array([np.random.uniform(self.xlim[0], self.xlim[1]),
                        np.random.uniform(self.ylim[0], self.ylim[1])])

        self.ctrl.limb.set_joint_position_speed(.2)

        close_nstep, tclose, up_nstep, t_up = 0, 0, 0, 0
        next_wypt = np.array([des_pos[0], des_pos[1], lower_height])

        for i_ac in range(num_actions):

            start_joints = self.ctrl.limb.joint_angles()

            #sample action type:
            action = np.random.randint(0,4)

            noptions = 3
            np.random.choice(range(noptions), p=[0.6, 0.2, 0.2])


            print 'step ', i_ac

            if i_ac > 0:
                if action == 0:
                    maxshift = .1
                    des_pos += np.random.uniform(-maxshift, maxshift, 2)
                    des_pos = self.truncate_pos(des_pos)# make sure not outside defined region
                    next_wypt[0] = des_pos[0]
                    next_wypt[1] = des_pos[1]
                    print 'move to ', next_wypt

                elif action == 1:   # close gripper and hold for n steps
                    close_nstep = np.random.randint(0, 6)
                    print 'close and hold for ', close_nstep
                    tclose = i_ac
                    self.ctrl.gripper.close()
                    gripper_closed = True

                elif action == 2:  # go up for n steps
                    up_nstep = np.random.randint(0, 6)
                    t_up = i_ac
                    delta_up = .1
                    next_wypt[2] = lower_height + delta_up
                    gripper_up = True
                    print 'up!'

            if gripper_closed:
                if i_ac > (tclose + close_nstep):
                    self.ctrl.gripper.open()

                    print 'opening gripper'

                gripper_closed = False

            if gripper_up:
                if i_ac > (t_up + up_nstep):
                    next_wypt[2] = lower_height
                    print 'going down'
                    gripper_up = False


            print 'holding force', self.ctrl.gripper.get_force()


            desired_pose = inverse_kinematics.get_pose_stamped(next_wypt[0],
                                                               next_wypt[1],
                                                               next_wypt[2],
                                                               inverse_kinematics.EXAMPLE_O)
            try:
                cmd = inverse_kinematics.get_joint_angles(desired_pose, seed_cmd=start_joints,
                                                          use_advanced_options=True)
            except ValueError:
                rospy.logerr('no inverse kinematics solution found, '
                             'going to reset robot...')
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

            self.recorder.take_shot(i_tr=i_ac)

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
