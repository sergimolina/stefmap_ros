#!/usr/bin/env python

import sys
import rospy
from stefmap_ros.srv import GetSTeFMap
from std_msgs.msg import Header

if __name__ == "__main__":

    rospy.init_node("stefmap_client_node")
    rospy.sleep(5)

    order = 10

    prediction_time = 0
    rospy.wait_for_service('get_stefmap')
    while (not rospy.is_shutdown()):
        try:
            print "TIME: "+str(prediction_time/3600)+":00h"
            get_stefmap = rospy.ServiceProxy('get_stefmap', GetSTeFMap)
            stefmap = get_stefmap(prediction_time,order)

            prediction_time = prediction_time + 3600

            if prediction_time == 3600*24:
                prediction_time = 0
                break            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e 
        rospy.sleep(2)
