#!/usr/bin/env python

import sys
import rospy
from stefmap_ros.srv import GetSTeFMap

if __name__ == "__main__":
    prediction_time = 1352265000
    order = 2
    x_min = -45
    x_max = 55
    y_min = -35
    y_max = 30
    cell_size = 1

    print "Requesting STeF-Map with the following parameters:\n"
    print "  Time to predict: %s"%(prediction_time)
    print "  Fremen Order: %s"%(order)
    print "  X min: %s"%(x_min)
    print "  X max: %s"%(x_max)
    print "  Y min: %s"%(y_min)
    print "  Y max: %s"%(y_max)
    print "  Cell size:%s"%(cell_size)

    rospy.wait_for_service('get_stefmap')
    try:
        get_stefmap = rospy.ServiceProxy('get_stefmap', GetSTeFMap)
        stefmap = get_stefmap(prediction_time,order,x_min,x_max,y_min,y_max,cell_size)
        print stefmap 
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e 

