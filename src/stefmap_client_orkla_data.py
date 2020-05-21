#!/usr/bin/env python

import sys
import rospy
from stefmap_ros.srv import GetSTeFMap
from std_msgs.msg import Header
import time
from dynamic_reconfigure.server import Server
from stefmap_ros.cfg import stefmapclientConfig
from datetime import datetime


def callback(config,level):

    if config.start_predictions:
        prediction_time = config.time_16_09_2019*3600 + 1560556800
        xmin = -20 # meters
        xmax = 60   # meters
        ymin = -5 # meters
        ymax = 65 # meters
        cell_size = 1 #meters
        order = config.model_order

        print "Requesting STeF-Map with the following parameters:\n"
        print "  Time to predict: %s"%(datetime.utcfromtimestamp(prediction_time).strftime('%Y-%m-%d %H:%M:%S'))
        print "  Fremen model Order: %s"%(order)
        print "  X min: %s"%(xmin)
        print "  X max: %s"%(xmax)
        print "  Y min: %s"%(ymin)
        print "  Y max: %s"%(ymax)
        print "  Cell size:%s"%(cell_size)

        rospy.wait_for_service('get_stefmap')
        
        try:
           #print(time.ctime(int(prediction_time)))
           get_stefmap = rospy.ServiceProxy('get_stefmap', GetSTeFMap)
           h = Header()
           h.stamp.secs = prediction_time
           h.frame_id = 'stefmap_frame'
           stefmap = get_stefmap(h,order,xmin,xmax,ymin,ymax,cell_size)
       
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e 

    return config


if __name__ == "__main__":
    rospy.init_node("stefmap_client", anonymous = False)

    srv = Server(stefmapclientConfig, callback)
    rospy.spin()
