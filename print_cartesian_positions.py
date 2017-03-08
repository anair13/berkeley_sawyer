#!/usr/bin/env python

# Copyright (c) 2016, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Intera RSDK Forward Kinematics Example
"""
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionFK,
    SolvePositionFKRequest,
)

import intera_interface

class Print_fwd_kin(object):
    def __init__(self, rate):

        print("Initializing node... ")
        rospy.init_node("cartestian_joint_printer")

        self.limb = intera_interface.Limb("right")
        # rospy.Subscriber("/robot/joint_states", JointState, self.get_latest_jointstate)

        limb = 'right'
        self.name_of_service = "ExternalTools/" + limb + "/PositionKinematicsNode/FKService"
        self.fksvc = rospy.ServiceProxy(self.name_of_service, SolvePositionFK)

        # self.latest_jointstate = None

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / rate), self.run_fwd_kin)


    # def get_latest_jointstate(self, data):
    #     self.latest_jointstate = data


    def run_fwd_kin(self, event):

        fkreq = SolvePositionFKRequest()
        joints = JointState()
        joints.name = self.limb.joint_names()

        joints.position = [self.limb.joint_angle(j)
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

        # Check if result valid
        if (resp.isValid[0]):
            rospy.loginfo("\nFK Joint Angles:\n")
            for i, name in enumerate(joints.name):
                print name + " %f"%(joints.position[i])
            rospy.loginfo("\nFK Cartesian Solution:\n")
            rospy.loginfo("------------------")
            rospy.loginfo("Response Message:\n%s", resp)
        else:
                rospy.loginfo("INVALID JOINTS - No Cartesian Solution Found.")

def main():
    """
    printing the
    response of whether a valid Cartesian solution was found,
    and if so, the corresponding Cartesian pose.
    """

    joint_printer = Print_fwd_kin(rate= 1)
    rospy.spin()



if __name__ == '__main__':
    main()
