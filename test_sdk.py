#!/usr/bin/env python

import robot_controller

if __name__ == '__main__':
    # main()
    r = robot_controller.RobotController()
    r.set_neutral()
    for i in range(5):
        n = input()
        j = r.joints[n]
        r.set_joint_delta(j, 0.1)
