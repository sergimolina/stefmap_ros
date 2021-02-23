#!/usr/bin/env python

import sys
import rospy
from stefmap_ros.srv import GetSTeFMap

class stefmap_client_node(object):

	def __init__(self):
		#parameters
		self.prediction_order = rospy.get_param('~prediction_order',0)
		self.time_between_predictions = rospy.get_param('~time_between_predictions',60)

		rospy.loginfo("Waiting for get_stefmap service...")
		rospy.wait_for_service('get_stefmap')
		self.get_stefmap = rospy.ServiceProxy('get_stefmap', GetSTeFMap)
		rospy.loginfo("Stefmap server active")
		
		self.prediction_timer = rospy.Timer(rospy.Duration(self.time_between_predictions),self.perform_prediction)
		
		self.run()
		

	def perform_prediction(self,timer):
	 	t = rospy.get_time()
		rospy.loginfo("Predicting stefmap at time: "+str(t))
		stefmap = self.get_stefmap(t,self.prediction_order)


	def run(self):
		while not rospy.is_shutdown():
			rospy.spin()

if __name__ == '__main__':
	rospy.init_node('stefmap_client_node', anonymous=True)
	scn = stefmap_client_node()
