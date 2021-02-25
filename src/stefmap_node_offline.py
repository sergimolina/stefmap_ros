#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from fremenarray.msg import FremenArrayActionGoal, FremenArrayGoal, FremenArrayAction
from geometry_msgs.msg import PoseArray, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
import actionlib
import math
import numpy as np
import tf
from stefmap_ros.msg import STeFMapMsg,STeFMapCellMsg
from stefmap_ros.srv import GetSTeFMap, UpdateSTeFMap

class STeFmap_node_offline(object):

	def __init__(self):
		#parameters
		self.grid_size = rospy.get_param('~grid_size',1)#meters		
		self.x_min = rospy.get_param('~x_min',-50)#meters
		self.x_max = rospy.get_param('~x_max',50) #meters
		self.y_min = rospy.get_param('~y_min',-50)#meters
		self.y_max = rospy.get_param('~y_max',50) #meters
		self.num_bins = rospy.get_param('~num_bins',8) #bins dividing the circumference
		self.frame_id = rospy.get_param('~frame_id',"/map")

		# initialize visibility map
		self.width = int((self.x_max-self.x_min)/self.grid_size) ## [cells]
		self.height = int((self.y_max-self.y_min)/self.grid_size) ## [cells]

		#initialize entropy map
		self.entropy_map = OccupancyGrid()
		self.entropy_map.header.frame_id = self.frame_id
		self.entropy_map.info.resolution = self.grid_size ## [m/cell]
		self.entropy_map.info.width = self.width ## [cells]
		self.entropy_map.info.height = self.height ## [cells]
		self.entropy_map.info.origin.position.x = self.x_min #origin of the map [m,m,rad]. This is the real #world pose of the cell (0,0) in the map
		self.entropy_map.info.origin.position.y = self.y_min 
		self.entropy_map.info.origin.position.z = 0
		self.entropy_map.info.origin.orientation.x = 0
		self.entropy_map.info.origin.orientation.y = 0
		self.entropy_map.info.origin.orientation.z = 0
		self.entropy_map.info.origin.orientation.w = 1
		self.entropy_map.data = np.zeros(self.width*self.height)

		# connect to fremenarray
		rospy.loginfo("waiting for FremenArray.....")
		self.fremenarray_client = actionlib.SimpleActionClient('/fremenarray', FremenArrayAction)
		self.fremenarray_client.wait_for_server()
		rospy.loginfo("FremenArray ready!")

		# initiliatize stefmap
		rospy.loginfo("Initializing STeF-map.....")
		self.bin_counts_matrix = np.ones([self.width,self.height,self.num_bins])*-1 #initialize cells as unexplored/unknow
		self.bin_counts_matrix_accumulated = np.zeros([self.width,self.height,self.num_bins])
		self.entropy_map_store = []

		fremenarray_msg=FremenArrayGoal()
		fremenarray_msg.operation = 'add'
		fremenarray_msg.time = 0
		fremenarray_msg.states = np.reshape(self.bin_counts_matrix,self.width*self.height*self.num_bins)
		self.fremenarray_client.send_goal(fremenarray_msg)
		self.fremenarray_client.wait_for_result()
		fremenarray_result = self.fremenarray_client.get_result()
		rospy.loginfo(fremenarray_result.message)
		rospy.loginfo("STeF-map ready!")

		# create stef map service and topic
		get_stefmap_service = rospy.Service('get_stefmap', GetSTeFMap, self.handle_GetSTeFMap)
		update_stefmap_service = rospy.Service('update_stefmap', UpdateSTeFMap, self.handle_UpdateSTeFMap)
		self.entropy_map_pub = rospy.Publisher('/entropy_map', OccupancyGrid, queue_size=1,latch=True)
		self.stefmap_pub = rospy.Publisher('/stefmap', STeFMapMsg, queue_size=1, latch=True)

		self.run()

		
	def handle_UpdateSTeFMap(self,req):
		self.bin_counts_matrix = np.reshape(req.data,[self.width,self.height,self.num_bins])
		self.bin_counts_matrix_accumulated = self.bin_counts_matrix_accumulated + self.bin_counts_matrix
		self.update_entropy_map(req.update_time)
		
		#normalize the histogram before update fremen in case they are not
		for r in range(0,int(self.width)):
			for c in range(0,int(self.height)):
				max_count = np.amax(self.bin_counts_matrix[r][c][:])
				if max_count > 0:
					for b in range(0,int(self.num_bins)):
						self.bin_counts_matrix[r][c][b] = 100*self.bin_counts_matrix[r][c][b]/max_count
		
		fremenarray_msg = FremenArrayGoal()
		fremenarray_msg.operation = 'add'
		fremenarray_msg.time = req.update_time
		fremenarray_msg.states = np.reshape(self.bin_counts_matrix,self.width*self.height*self.num_bins)

		self.fremenarray_client.send_goal(fremenarray_msg)
		self.fremenarray_client.wait_for_result()
		fremenarray_result = self.fremenarray_client.get_result()
		rospy.loginfo(fremenarray_result.message)
		return fremenarray_result.message

	def update_entropy_map(self,update_time):
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
			print "Error getting the STeF-Map"
			mSTefMap = STeFMapMsg()
			return mSTefMap
		fremenarray_result = self.fremenarray_client.get_result()

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
		print("STeFMap sent!")
		return mSTefMap

	def run(self):				
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('stefmap_node', anonymous=True)
	stef = STeFmap_node_offline()
