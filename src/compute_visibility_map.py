#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseArray, PointStamped,PoseStamped
from nav_msgs.msg import OccupancyGrid
import actionlib
import math
import numpy as np
import tf
from stefmap_ros.srv import GetVisibilityMap
import cell_tools as ctools

class compute_visibility_map_node(object):

	def __init__(self):
		#parameters
		self.grid_size = rospy.get_param('~grid_size',1)#meters		
		self.x_min = rospy.get_param('~x_min',-50)#meters
		self.x_max = rospy.get_param('~x_max',50) #meters
		self.y_min = rospy.get_param('~y_min',-50)#meters
		self.y_max = rospy.get_param('~y_max',50) #meters
		self.coverage_laser_topic_1 = rospy.get_param('~coverage_laser_topic_1',"/coverage_scan_1")
		self.coverage_laser_topic_2 = rospy.get_param('~coverage_laser_topic_2',"/coverage_scan_2")
		self.coverage_laser_topic_3 = rospy.get_param('~coverage_laser_topic_3',"/coverage_scan_3")
		self.coverage_laser_topic_4 = rospy.get_param('~coverage_laser_topic_4',"/coverage_scan_4")
		self.coverage_laser_topic_5 = rospy.get_param('~coverage_laser_topic_5',"/coverage_scan_5")
		self.coverage_laser_topic_6 = rospy.get_param('~coverage_laser_topic_6',"/coverage_scan_6")
		self.coverage_laser_topic_7 = rospy.get_param('~coverage_laser_topic_7',"/coverage_scan_7")
		self.coverage_laser_topic_8 = rospy.get_param('~coverage_laser_topic_8',"/coverage_scan_8")
		self.coverage_laser_topic_9 = rospy.get_param('~coverage_laser_topic_9',"/coverage_scan_9")
		self.coverage_time_update = rospy.get_param('~coverage_time_update',5) # seconds
		self.max_coverage_distance = rospy.get_param('~max_coverage_distance',15) # meters
		self.frame_id = rospy.get_param('~frame_id',"map")
		self.full_observability  = rospy.get_param('~full_observability',False)
		
		#ini variables
		self.width = int((self.x_max-self.x_min)/self.grid_size) ## [cells]
		self.height = int((self.y_max-self.y_min)/self.grid_size) ## [cells]

		self.laser_data_received = [0,0,0,0,0,0,0,0,0]

		# visibility map_msg
		self.visibility_map = OccupancyGrid()
		self.visibility_map.header.frame_id = self.frame_id
		self.visibility_map.info.resolution = self.grid_size ## [m/cell]
		self.visibility_map.info.width = self.width ## [cells]
		self.visibility_map.info.height = self.height ## [cells]
		self.visibility_map.info.origin.position.x = self.x_min #origin of the map [m,m,rad]. This is the real #world pose of the cell (0,0) in the map
		self.visibility_map.info.origin.position.y = self.y_min 
		self.visibility_map.info.origin.position.z = 0
		self.visibility_map.info.origin.orientation.x = 0
		self.visibility_map.info.origin.orientation.y = 0
		self.visibility_map.info.origin.orientation.z = 0
		self.visibility_map.info.origin.orientation.w = 1
		self.visibility_map.data = np.zeros(self.width*self.height)#The map data, in row-major order, starting with (0,0)

		self.tf_listener = tf.TransformListener()

		# subscribe to topics
		rospy.Subscriber(self.coverage_laser_topic_1,LaserScan, self.lasercoverage_callback_1,queue_size=1)
		rospy.Subscriber(self.coverage_laser_topic_2,LaserScan, self.lasercoverage_callback_2,queue_size=1)
		rospy.Subscriber(self.coverage_laser_topic_3,LaserScan, self.lasercoverage_callback_3,queue_size=1)
		rospy.Subscriber(self.coverage_laser_topic_4,LaserScan, self.lasercoverage_callback_4,queue_size=1)
		rospy.Subscriber(self.coverage_laser_topic_5,LaserScan, self.lasercoverage_callback_5,queue_size=1)
		rospy.Subscriber(self.coverage_laser_topic_6,LaserScan, self.lasercoverage_callback_6,queue_size=1)
		rospy.Subscriber(self.coverage_laser_topic_7,LaserScan, self.lasercoverage_callback_7,queue_size=1)
		rospy.Subscriber(self.coverage_laser_topic_8,LaserScan, self.lasercoverage_callback_8,queue_size=1)
		rospy.Subscriber(self.coverage_laser_topic_9,LaserScan, self.lasercoverage_callback_9,queue_size=1)

		# create topic publishers
		self.visibility_map_pub = rospy.Publisher('/visibility_map', OccupancyGrid, queue_size=1)

		# create services
		visibility_map_service = rospy.Service('get_visibility_map', GetVisibilityMap, self.handle_get_visibility_map)

		# create timers
		self.update_visibility_timer = rospy.Timer(rospy.Duration(self.coverage_time_update),self.update_visiblity_map)

		self.run()

	def lasercoverage_callback_1(self,laser_data):
		self.last_laser_data_1 = laser_data
		self.laser_data_received[0] = 1

	def lasercoverage_callback_2(self,laser_data):
		self.last_laser_data_2 = laser_data
		self.laser_data_received[1] = 1

	def lasercoverage_callback_3(self,laser_data):
		self.last_laser_data_3 = laser_data
		self.laser_data_received[2] = 1

	def lasercoverage_callback_4(self,laser_data):
		self.last_laser_data_4 = laser_data
		self.laser_data_received[3] = 1

	def lasercoverage_callback_5(self,laser_data):
		self.last_laser_data_5 = laser_data
		self.laser_data_received[4] = 1

	def lasercoverage_callback_6(self,laser_data):
		self.last_laser_data_6 = laser_data
		self.laser_data_received[5] = 1

	def lasercoverage_callback_7(self,laser_data):
		self.last_laser_data_7 = laser_data
		self.laser_data_received[6] = 1

	def lasercoverage_callback_8(self,laser_data):
		self.last_laser_data_8 = laser_data
		self.laser_data_received[7] = 1

	def lasercoverage_callback_9(self,laser_data):
		self.last_laser_data_9 = laser_data
		self.laser_data_received[8] = 1

	def update_visiblity_map(self,timer):
		# get the laser position in map coordinates
		if self.full_observability == False:
			self.visibility_map.data = np.zeros(self.width*self.height) #The map data, in row-major order, starting with (0,0)
			for l in range(0,len(self.laser_data_received)):
				if self.laser_data_received[l]:
					if l == 0:
						self.last_laser_data = self.last_laser_data_1
					elif l == 1:
						self.last_laser_data = self.last_laser_data_2
					elif l == 2:
						self.last_laser_data = self.last_laser_data_3
					elif l == 3:
						self.last_laser_data = self.last_laser_data_4
					elif l == 4:
						self.last_laser_data = self.last_laser_data_5
					elif l == 5:
						self.last_laser_data = self.last_laser_data_6
					elif l == 6:
						self.last_laser_data = self.last_laser_data_7
					elif l == 7:
						self.last_laser_data = self.last_laser_data_8
					elif l == 8:
						self.last_laser_data = self.last_laser_data_9

					start = time.time()

					try:
						(trans,rot) = self.tf_listener.lookupTransform(self.frame_id, self.last_laser_data.header.frame_id, rospy.Time(0))
					except:
						rospy.loginfo("TF transform between "+self.frame_id+" and "+self.last_laser_data.header.frame_id+" not found")
						return

					laser_sensor_x = trans[0]
					laser_sensor_y = trans[1]
					[temp,temp,laser_sensor_angle] = euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
					
					#print self.last_laser_data.range_min,self.last_laser_data.range_max
					for i in range(0,len(self.last_laser_data.ranges)):
						current_angle = self.last_laser_data.angle_min + self.last_laser_data.angle_increment*i

						if np.isinf(self.last_laser_data.ranges[i]) or (self.last_laser_data.ranges[i] > self.max_coverage_distance):
							#point in laser coordinatesranges
							laser_point_x = self.max_coverage_distance * np.cos(current_angle)
							laser_point_y = self.max_coverage_distance * np.sin(current_angle)
						else:
							laser_point_x = self.last_laser_data.ranges[i] * np.cos(current_angle)
							laser_point_y = self.last_laser_data.ranges[i] * np.sin(current_angle)		

						# transform the point in map coordinates
						laser_point=PointStamped()
						laser_point.header.frame_id = self.last_laser_data.header.frame_id
						laser_point.header.stamp =rospy.Time(0)
						laser_point.point.x=laser_point_x
						laser_point.point.y=laser_point_y
						laser_point.point.z=0.0
						laser_point_map=self.tf_listener.transformPoint(self.frame_id,laser_point)

						# calculate several points in the ray trace
						raytrace_x_points = np.linspace(laser_sensor_x,laser_point_map.point.x,100) # divide the trace in 100 segments (this could be a parameter)
						raytrace_y_points = np.linspace(laser_sensor_y,laser_point_map.point.y,100)
						
						#find the associated cells to the ray trace points found
						for j in range(0,len(raytrace_x_points)):
							index = ctools.point2index(raytrace_x_points[j],raytrace_y_points[j],self.x_min,self.x_max,self.y_min,self.y_max,self.grid_size,self.width,self.height)
							if index != -1:
								self.visibility_map.data[index] = 100

					end = time.time()
					#rospy.loginfo("Laserscan num "+str(l+1)+" has taken " + str(end - start)+" seconds to compute its visibility map")
		else:
			self.visibility_map.data = np.ones(self.width*self.height)*100

		self.visibility_map_pub.publish(self.visibility_map)
		#self.last_visibility_map = self.visibility_map
		return self.visibility_map

	def handle_get_visibility_map(self,req):
		return	self.visibility_map
		
	def run(self):		
		while not rospy.is_shutdown():
			rospy.spin()


if __name__ == '__main__':
	rospy.init_node('compute_visibility_map_node', anonymous=True)
	cvmn = compute_visibility_map_node()
