#!/usr/bin/env python

import sys
import rospy
from berkeley_sawyer.srv import *
import pdb
import numpy as np
import rospy.numpy_msg

import cv2
from cv_bridge import CvBridge, CvBridgeError

def test_service():
    print 'waiting for service ..'
    rospy.wait_for_service('get_action')
    while True:
        print 'looping ..'

        try:
            get_action_func = rospy.ServiceProxy('get_action', get_kinectdata)

            a = np.ones(4)

            # get_action._request_class = rospy.numpy_msg.numpy_msg(get_actionRequest)
            req = get_actionRequest

            req.images = a
            # req = rospy.numpy_msg.numpy_msg(req)
            resp1 = get_action_func(req)
            print 'service call succeeded'
            pdb.set_trace()

            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == "__main__":
    print 'main'
    rospy.init_node('testclient')
    test_service()