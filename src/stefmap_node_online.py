#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from fremenarray.msg import FremenArrayActionGoal, FremenArrayGoal, FremenArrayAction
from geometry_msgs.msg import PoseArray, PointStamped,PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
import actionlib
import math
import numpy as np
import tf
from stefmap_ros.msg import STeFMapMsg,STeFMapCellMsg
from stefmap_ros.srv import GetSTeFMap, GetVisibilityMap
import cell_tools as ctools

class STeFmap_node_online(object):

	def __init__(self):
		#parameters
		self.grid_size = rospy.get_param('~grid_size',1)#meters		
		self.x_min = rospy.get_param('~x_min',-50)#meters
		self.x_max = rospy.get_param('~x_max',50) #meters
		self.y_min = rospy.get_param('~y_min',-50)#meters
		self.y_max = rospy.get_param('~y_max',50) #meters
		self.interval_time = rospy.get_param('~interval_time',600)#seconds
		self.num_bins = rospy.get_param('~num_bins',8) #bins dividing the circumference
		self.frame_id = rospy.get_param('~frame_id',"map")
		self.load_data = rospy.get_param('~load_data',False) 
		self.data_to_load_file = rospy.get_param('~data_to_load_file',"test_data.txt") # meters
		self.save_histograms = rospy.get_param('~save_histograms',False) # meters
		self.histograms_to_save_file = rospy.get_param('~histograms_to_save_file',"test_histograms.txt") # meters
		self.people_detections_topic_1 = rospy.get_param('~people_detections_topic_1',"/people_detections_1")
		self.people_detections_topic_2 = rospy.get_param('~people_detections_topic_2',"/people_detections_2")
		self.people_detections_topic_3 = rospy.get_param('~people_detections_topic_3',"/people_detections_3")
		self.people_detections_topic_4 = rospy.get_param('~people_detections_topic_4',"/people_detections_4")
		self.people_detections_topic_5 = rospy.get_param('~people_detections_topic_5',"/people_detections_5")
		self.people_detections_topic_6 = rospy.get_param('~people_detections_topic_6',"/people_detections_6")
		self.people_detections_topic_7 = rospy.get_param('~people_detections_topic_7',"/people_detections_7")
		self.people_detections_topic_8 = rospy.get_param('~people_detections_topic_8',"/people_detections_8")
		self.people_detections_topic_9 = rospy.get_param('~people_detections_topic_9',"/people_detections_9")
		
		#variables
		self.width = int((self.x_max-self.x_min)/self.grid_size) ## [cells]
		self.height = int((self.y_max-self.y_min)/self.grid_size) ## [cells]

		self.tf_listener = tf.TransformListener()

		# connect to fremenarray
		rospy.loginfo("waiting for FremenArray.....")
		self.fremenarray_client = actionlib.SimpleActionClient('/fremenarray', FremenArrayAction)
		self.fremenarray_client.wait_for_server()
		rospy.loginfo("FremenArray ready!")

		# initiliatize stefmap
		rospy.loginfo("Initializing STeF-map.....")
		self.bin_counts_matrix = np.ones([self.width,self.height,self.num_bins])*-1 #initialize cells as unexplored/unknow
		self.bin_counts_matrix_accumulated = np.zeros([self.width,self.height,self.num_bins])

		fremenarray_msg=FremenArrayGoal()
		fremenarray_msg.operation = 'add'
		fremenarray_msg.time = 0
		fremenarray_msg.states = np.reshape(self.bin_counts_matrix,self.width*self.height*self.num_bins)
		self.fremenarray_client.send_goal(fremenarray_msg)
		self.fremenarray_client.wait_for_result()
		fremenarray_result = self.fremenarray_client.get_result()
		rospy.loginfo(fremenarray_result.message)
		rospy.loginfo("STeF-map ready!")

		# create topic publishers
		self.entropy_map_pub = rospy.Publisher('/entropy_map', OccupancyGrid, queue_size=1,latch=True)
		self.stefmap_pub = rospy.Publisher('/stefmap', STeFMapMsg, queue_size=1, latch=True)

		# create services
		stefmap_service = rospy.Service('get_stefmap', GetSTeFMap, self.handle_GetSTeFMap)
		
		# Before creating the timers to start updating the fremen models, check wheter there is some data to load
		if self.load_data:
			rospy.loginfo("Starting to load data")
			with open(self.data_to_load_file,"r") as file:
				for line in file:
					current_line = line.split(',')
					timestamp = int(float(current_line[0]))
					states = []
					for i in range(1,len(current_line)):
						states.append(int(current_line[i]))

					try:
						self.bin_counts_matrix = np.reshape(states,[self.width,self.height,self.num_bins])
					except:
						rospy.loginfo("Data to load dimension is not the same as the dimension defined for the stefmap")
						continue
					self.bin_counts_matrix_accumulated = self.bin_counts_matrix_accumulated + self.bin_counts_matrix
					self.update_entropy_map()

					#normalize the histogram before update to fremen
					for r in range(0,int(self.width)):
						for c in range(0,int(self.height)):
							max_count = np.amax(self.bin_counts_matrix[r][c][:])
							if max_count > 0:
								for b in range(0,int(self.num_bins)):
									self.bin_counts_matrix[r][c][b] = 100*self.bin_counts_matrix[r][c][b]/max_count

					fremenarray_msg=FremenArrayGoal()
					fremenarray_msg.operation = 'add'
					fremenarray_msg.time = timestamp
					fremenarray_msg.states = np.reshape(self.bin_counts_matrix,self.width*self.height*self.num_bins)
					self.fremenarray_client.send_goal(fremenarray_msg)
					self.fremenarray_client.wait_for_result()
					fremenarray_result = self.fremenarray_client.get_result()
					rospy.loginfo(fremenarray_result.message)
				
			rospy.loginfo("All data loaded")

		# subscribe to topics
		rospy.Subscriber("/visibility_map", OccupancyGrid, self.visibility_map_callback,queue_size=1)
		self.visibility_map_received = False

		rospy.Subscriber(self.people_detections_topic_1, PoseArray, self.people_detections_callback,queue_size=10)
		rospy.Subscriber(self.people_detections_topic_2, PoseArray, self.people_detections_callback,queue_size=10)
		rospy.Subscriber(self.people_detections_topic_3, PoseArray, self.people_detections_callback,queue_size=10)
		rospy.Subscriber(self.people_detections_topic_4, PoseArray, self.people_detections_callback,queue_size=10)
		rospy.Subscriber(self.people_detections_topic_5, PoseArray, self.people_detections_callback,queue_size=10)
		rospy.Subscriber(self.people_detections_topic_6, PoseArray, self.people_detections_callback,queue_size=10)
		rospy.Subscriber(self.people_detections_topic_7, PoseArray, self.people_detections_callback,queue_size=10)
		rospy.Subscriber(self.people_detections_topic_8, PoseArray, self.people_detections_callback,queue_size=10)
		rospy.Subscriber(self.people_detections_topic_9, PoseArray, self.people_detections_callback,queue_size=10)




		# create timers
		self.update_fremen_timer = rospy.Timer(rospy.Duration(self.interval_time),self.update_fremen_models)
		
		self.run()

	def people_detections_callback(self, data):
		if self.visibility_map_received:
			for i in range(0,len(data.poses)):
				try:
					current_pose = PoseStamped()
					current_pose.header = data.header
					current_pose.pose = data.poses[i]
					pose_in_map = self.tf_listener.transformPose(self.frame_id,current_pose)
				except Exception as e:
					rospy.loginfo("TF transform between "+self.frame_id+" and "+data.header.frame_id+" not found")
					return
				# calculate the cell and the angle_bin where the detection belongs
				(cell_x,cell_y) = ctools.point2cell(pose_in_map.pose.position.x,pose_in_map.pose.position.y,self.x_min,self.x_max,self.y_min,self.y_max,self.grid_size)
				(roll, pitch, yaw) = euler_from_quaternion ([pose_in_map.pose.orientation.x,pose_in_map.pose.orientation.y,pose_in_map.pose.orientation.z,pose_in_map.pose.orientation.w])
				
				if cell_x != -1 and cell_y !=1:# self.width and cell_y >= 0 and cell_y < self.height:
					#print "detection within stefmap boundaries"
					if self.visibility_map.data[ctools.cell2index(cell_x,cell_y,self.width,self.height)] == 100:
						#calculate the bin
						if yaw < 0:
							angle_bin = math.ceil((math.degrees(yaw)+360+(180/self.num_bins))/(360/self.num_bins)) - 1
							if angle_bin == self.num_bins:
								angle_bin = 0
						else:
							angle_bin = math.ceil((math.degrees(yaw)+(180/self.num_bins))/(360/self.num_bins)) - 1

						#update the count with 1
						self.bin_counts_matrix[cell_x][cell_y][int(angle_bin)] = self.bin_counts_matrix[cell_x][cell_y][int(angle_bin)] + 1
						self.bin_counts_matrix_accumulated[cell_x][cell_y][int(angle_bin)] = self.bin_counts_matrix_accumulated[cell_x][cell_y][int(angle_bin)] + 1
					#else:
					#	print "detection out of robot range"

				#else:
				#	print "detection out of boundaries"

	def update_fremen_models(self,timer):
		if self.visibility_map_received == False:
			rospy.loginfo("No visibility_map received yet")

		#before normalizing the counts, update the spatial entropy model
		bin_counts_matrix_normalised = self.bin_counts_matrix
		self.bin_counts_matrix = np.ones([self.width,self.height,self.num_bins])*-1 #initialize cells as unexplored/unknow

		cells_visible = 0
		# Normalize the count matrix for each cell, giving the orientation with the maximum counts the max value
		for r in range(0,int(self.width)):
			for c in range(0,int(self.height)):
				max_count = np.amax(bin_counts_matrix_normalised[r][c][:])
				if max_count > 0:
					for b in range(0,int(self.num_bins)):
						bin_counts_matrix_normalised[r][c][b] = 100*bin_counts_matrix_normalised[r][c][b]/max_count
				if max_count == 0:
					cells_visible = cells_visible  + 1

		fremenarray_msg = FremenArrayGoal()
		fremenarray_msg.operation = 'add'
		fremenarray_msg.time = rospy.get_time()
		fremenarray_msg.states = np.reshape(bin_counts_matrix_normalised,self.width*self.height*self.num_bins)

		self.fremenarray_client.send_goal(fremenarray_msg)
		self.fremenarray_client.wait_for_result()
		fremenarray_result = self.fremenarray_client.get_result()
		rospy.loginfo(fremenarray_result.message)
		self.update_entropy_map()

		if self.save_histograms:
			with open(self.histograms_to_save_file,"a") as hfile:
				hfile.write(str(fremenarray_msg.time))
				hfile.write(",")
				for j in range(0,self.width*self.height*self.num_bins):
					hfile.write(str(int(fremenarray_msg.states[j])))
					if j != self.width*self.height*self.num_bins-1:
						hfile.write(",")
				hfile.write("\n")




	def update_entropy_map(self):
		entropy_map = OccupancyGrid()
		entropy_map.header.frame_id = self.frame_id
		entropy_map.info.resolution = self.grid_size ## [m/cell]
		entropy_map.info.width = self.width ## [cells]
		entropy_map.info.height = self.height ## [cells]
		entropy_map.info.origin.position.x = self.x_min #origin of the map [m,m,rad]. This is the real #world pose of the cell (0,0) in the map
		entropy_map.info.origin.position.y = self.y_min 
		entropy_map.info.origin.position.z = 0
		entropy_map.info.origin.orientation.x = 0
		entropy_map.info.origin.orientation.y = 0
		entropy_map.info.origin.orientation.z = 0
		entropy_map.info.origin.orientation.w = 1
		entropy_map.data = np.zeros(self.width*self.height)

		for r in range(0,int(self.width)):
			for c in range(0,int(self.height)):
				index = ctools.cell2index(r,c,self.width,self.height)
				# normalized the accumulated distribution -> total sum = 1
				total_count = np.sum(self.bin_counts_matrix_accumulated[r][c][:])
				if total_count > 0:
					for b in range(0,int(self.num_bins)):
						if self.bin_counts_matrix_accumulated[r][c][b] > 0:
							p_b = self.bin_counts_matrix_accumulated[r][c][b]/total_count
							entropy_map.data[index] =  entropy_map.data[index] + (-p_b*np.log2(p_b))
					bias = ((self.num_bins-1)/(2*float(total_count)))*np.log2(2.718281)#e = 2.718281
					entropy_map.data[index] = int(math.ceil(10*(entropy_map.data[index]+bias))) #e = 2.718281
				else:
					entropy_map.data[index] = 10
		self.entropy_map_pub.publish(entropy_map)

	def visibility_map_callback(self,visibility_map_msg):
		self.visibility_map_received = True
		self.visibility_map = visibility_map_msg
		for i in range(0,len(visibility_map_msg.data)):
			if self.visibility_map.data[i] > 0:
				cell_x,cell_y = ctools.index2cell(i,self.width,self.height)
				if cell_x !=-1 and cell_y != -1:#>= 0 and cell_x < self.width and cell_y >= 0 and cell_y < self.height:
					if min(self.bin_counts_matrix[cell_x][cell_y][:]) == -1:
						for b in range(0,self.num_bins):
							self.bin_counts_matrix[cell_x][cell_y][b] = 0

	def handle_GetSTeFMap(self,req):
		mSTefMap = STeFMapMsg()
		mSTefMap.header.stamp = rospy.get_rostime() 
		mSTefMap.header.frame_id = self.frame_id
		mSTefMap.prediction_time = req.prediction_time
		mSTefMap.x_min = self.x_min
		mSTefMap.x_max = self.x_max
		mSTefMap.y_min = self.y_min
		mSTefMap.y_max = self.y_max
		mSTefMap.cell_size = self.grid_size
		mSTefMap.rows = self.width
		mSTefMap.columns = self.height

		fremenarray_msg = FremenArrayGoal()
		fremenarray_msg.operation = 'predict'
		fremenarray_msg.order = req.order
		fremenarray_msg.time = req.prediction_time

		self.fremenarray_client.send_goal(fremenarray_msg)
		if not self.fremenarray_client.wait_for_result(rospy.Duration(10.0)):
			rospy.loginfo("Error getting the STeF-Map")
			mSTefMap = STeFMapMsg()
			return mSTefMap
		fremenarray_result = self.fremenarray_client.get_result()
		rospy.loginfo(fremenarray_result.message)
		prob_matrix = np.reshape(fremenarray_result.probabilities,(self.width,self.height,self.num_bins))		

		# iterate through all the cell and get also the bin with the maximum probability and the associated angle
		index = 0
		for r in range(0,mSTefMap.rows):
			for c in range(0,mSTefMap.columns):
				stefmap_cell = STeFMapCellMsg()

				stefmap_cell.row = int(r)
				stefmap_cell.column = int(c)
				stefmap_cell.x = float(self.x_min + self.grid_size*0.5 + self.grid_size * r)
				stefmap_cell.y = float(self.y_min + self.grid_size*0.5 + self.grid_size * c)
				stefmap_cell.probabilities = [float(prob_matrix[r,c,0]) ,
											  float(prob_matrix[r,c,1]) ,
											  float(prob_matrix[r,c,2]) ,
											  float(prob_matrix[r,c,3]) ,
											  float(prob_matrix[r,c,4]) ,
											  float(prob_matrix[r,c,5]) ,
											  float(prob_matrix[r,c,6]) ,
											  float(prob_matrix[r,c,7]) ] 

				max_number = -1
				for b in range(0,self.num_bins):
					if prob_matrix[r,c,b] > max_number:
						max_number = prob_matrix[r,c,b]
						max_orientation = b

				stefmap_cell.best_angle = max_orientation*360/self.num_bins

				mSTefMap.cells.append(stefmap_cell)
				index = index + 1

		self.stefmap_pub.publish(mSTefMap)
		#print("STeFMap sent!")
		return mSTefMap

		
	def run(self):		
		while not rospy.is_shutdown():
			rospy.spin()


if __name__ == '__main__':
	rospy.init_node('stefmap_node', anonymous=True)
	stef = STeFmap_node_online()
