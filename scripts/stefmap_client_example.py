#!/usr/bin/env python

import sys
import rospy
from stefmap_ros.srv import GetSTeFMap
from std_msgs.msg import Header

if __name__ == "__main__":
    prediction_time = 0
    order = 0

    rospy.wait_for_service('get_stefmap')
    while (1):
        try:
            get_stefmap = rospy.ServiceProxy('get_stefmap', GetSTeFMap)
            stefmap = get_stefmap(prediction_time,order)
            #print stefmap 
        
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e 
        rospy.sleep(5)
