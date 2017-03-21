#!/usr/bin/env python

import sys
import rospy
from berkeley_sawyer.srv import *
import pdb

def test_service():
    print 'waiting for service ..'
    rospy.wait_for_service('get_kinectdata')
    while True:

        print 'looping ..'


        try:
            get_kinectdata_func = rospy.ServiceProxy('get_kinectdata', get_kinectdata)

            resp1 = get_kinectdata_func()
            print 'service call succeeded'
            pdb.set_trace()


            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        print 'service call succeeded'
        pdb.set_trace()

if __name__ == "__main__":
    print 'main'
    rospy.init_node('testclient')
    test_service()