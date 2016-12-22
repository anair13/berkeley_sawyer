#!/usr/bin/env python

import robot_controller
import robot_recorder

if __name__ == '__main__':
    # main()
    c = robot_controller.RobotController()
    r = robot_recorder.RobotRecorder("/media/guser/data/recordings/test.csv", 10)
    c.set_neutral()
    for i in range(5):
        n = input()
        j = c.joints[n]
        c.set_joint_delta(j, 0.1)
