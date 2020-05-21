#!/usr/bin/env python

import rospy
import sys
import time
from std_msgs.msg import String
import os
import actionlib
from stefmap_ros.srv import UpdateSTeFMap

if __name__ == '__main__':

	file_name = sys.argv[1]

	rospy.init_node('load_data_fremen', anonymous=True)
	rospy.wait_for_service('update_stefmap')

	states = []
	with open(file_name,"r") as file:
		for line in file:
			current_line = line.split(',')
			timestamp = int(float(current_line[0]))
			for i in range(1,len(current_line)):
				states.append(float(current_line[i]))

			update_stefmap = rospy.ServiceProxy('update_stefmap', UpdateSTeFMap)
			result = update_stefmap(timestamp,states)
			print result

			states=[]