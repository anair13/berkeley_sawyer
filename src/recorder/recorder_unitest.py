#!/usr/bin/env python

import sys
import rospy
from berkeley_sawyer.srv import *
import pdb

from recorder import robot_recorder

def unit_test():

    rec = robot_recorder.RobotRecorder(save_dir="/home/guser/sawyer_data/test_recording", start_loop=False)

    for tr in range(5):
        print 'init traj', tr

        rec.init_traj(tr)
        for i in range(20):
            rec.save
            pdb.set_trace()

if __name__ == "__main__":
    print 'main'
    rospy.init_node('rec_unittest')
    unit_test()