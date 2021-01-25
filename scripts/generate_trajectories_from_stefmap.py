#!/usr/bin/env python

import time
import math
import rospy
import random
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from stefmap_ros.msg import STeFMapMsg,STeFMapCellMsg
import json
import numpy as np

class trajectory_generator_node(object):

	def __init__(self):

		#params
		self.output_filename = "../data/trajectories_zone0_12h_scenario2.txt"
		self.stefmap_prediction_filename = "../data/orkla_stefmap_12am_order10.json"
		self.zone_number = 0
		self.zone_limits_filename = "../config/orkla_zone_limits.json"
		self.num_traj_to_generate = 10
		self.min_traj_length = 5 #cells
		self.max_traj_length = 20 #cells
		self.use_montercarlo = "true" #true/false

		#read the zone limits file
		with open(self.zone_limits_filename,"r") as zfile:
			self.zone_limits = json.load(zfile)

		#read the stefmap folder
		with open(self.stefmap_prediction_filename,"r") as ifile:
			self.stefmap_prediction = json.load(ifile)

		print self.stefmap_prediction["stefmap"]["x_min"]
		print self.stefmap_prediction["stefmap"]["x_min"]

		self.frame_id = self.stefmap_prediction["stefmap"]["header"]["frame_id"]
		self.x_min = self.stefmap_prediction["stefmap"]["x_min"]
		self.x_max = self.stefmap_prediction["stefmap"]["x_max"]
		self.y_min = self.stefmap_prediction["stefmap"]["y_min"]
		self.y_max = self.stefmap_prediction["stefmap"]["y_max"]
		self.cell_size = self.stefmap_prediction["stefmap"]["cell_size"]
		self.rows = self.stefmap_prediction["stefmap"]["rows"]
		self.columns = self.stefmap_prediction["stefmap"]["columns"]
		self.num_cells = len(self.stefmap_prediction["stefmap"]["cells"])

		self.width = int((self.x_max-self.x_min)/self.cell_size) ## [cells]
		self.height = int((self.y_max-self.y_min)/self.cell_size) ## [cells]


		# define publishers and subscribers
		self.markerPub = rospy.Publisher('generated_trajectory_marker', Marker,queue_size=10)
		self.stefmapPub = rospy.Publisher('stefmap_for_traj_generator',STeFMapMsg, queue_size=1, latch=True)

		#publish the stefmap used for the generator
		self.mSTefMap = STeFMapMsg()
		self.mSTefMap.header.stamp = rospy.get_rostime() 
		self.mSTefMap.header.frame_id = self.frame_id
		self.mSTefMap.prediction_time = 0 
		self.mSTefMap.x_min = self.x_min
		self.mSTefMap.x_max = self.x_max
		self.mSTefMap.y_min = self.y_min
		self.mSTefMap.y_max = self.y_max
		self.mSTefMap.cell_size = self.cell_size
		self.mSTefMap.rows = self.width
		self.mSTefMap.columns = self.height
		index = 0
		for r in range(0,self.rows):
			for c in range(0,self.columns):
				stefmap_cell = STeFMapCellMsg()
				stefmap_cell.row = self.stefmap_prediction["stefmap"]["cells"][index]["row"]
				stefmap_cell.column = self.stefmap_prediction["stefmap"]["cells"][index]["column"]
				stefmap_cell.x = self.stefmap_prediction["stefmap"]["cells"][index]["x"]
				stefmap_cell.y = self.stefmap_prediction["stefmap"]["cells"][index]["y"]
				stefmap_cell.probabilities = self.stefmap_prediction["stefmap"]["cells"][index]["probabilities"]
				stefmap_cell.best_angle = self.stefmap_prediction["stefmap"]["cells"][index]["best_angle"]
				self.mSTefMap.cells.append(stefmap_cell)
				index = index + 1
		self.stefmapPub.publish(self.mSTefMap)

		self.run()

	def point2index(self,point_x,point_y):
		cell_x,cell_y = self.point2cell(point_x,point_y)
		index = self.cell2index(cell_x,cell_y)
		return int(index)

	def point2cell(self,point_x,point_y):
		cell_x = math.floor((point_x - self.x_min)/self.cell_size)
		cell_y = math.floor((point_y - self.y_min)/self.cell_size)
		return int(cell_x),int(cell_y)

	def cell2index(self,cell_x,cell_y):
		index = cell_x + cell_y*self.width 
		return int(index)

	def index2cell(self,index):
		cell_y = index*(self.height-1)/(self.height*self.width-1)
		cell_x = index - cell_y*self.width	
		return int(cell_x),int(cell_y)

	def compute_trajectory(self):
		cell_path = []
		backwards_cell_path = []
		mov_path = []

		# 1 - chose a cell within the zone limits and with a prob higher than 0 to start the trajectory
		sum_prob = 0
		iterations = 0
		while sum_prob == 0 and iterations<10000:
			x_start = random.uniform(self.zone_limits[str(self.zone_number)]["x_min"],self.zone_limits[str(self.zone_number)]["x_max"])
			y_start = random.uniform(self.zone_limits[str(self.zone_number)]["y_min"],self.zone_limits[str(self.zone_number)]["y_max"])
			starting_cell = self.point2index(y_start,x_start)
			sum_prob = sum(self.stefmap_prediction["stefmap"]["cells"][starting_cell]["probabilities"])
			iterations = iterations + 1

		if iterations >=10000:
			print "no starting cell found"
			cell_path.append(starting_cell)
			return cell_path,backwards_cell_path

		# 1 - chose a random cell with prob higher than 0 to start the trajectory
		# sum_prob = 0
		# iterations = 0
		# while sum_prob == 0 and iterations<10000:
		# 	starting_cell = random.randint(0,self.num_cells-1)
		# 	sum_prob = sum(self.stefmap_prediction["stefmap"]["cells"][starting_cell]["probabilities"])
		# 	iterations = iterations + 1

		# if iterations >=10000:
		# 	print "no starting cell found"
			# return cell_path

		#print "starting cell:", starting_cell
		cell_path.append(starting_cell)


		# 2 - compute the trajectory path forward
		mov_direction = int(self.stefmap_prediction["stefmap"]["cells"][starting_cell]["best_angle"]/45)
		mov_path.append(mov_direction)
		
		current_cell = self.calculate_destination_cell(starting_cell,mov_direction)

		if current_cell != -1: # valid cell
			cell_path.append(current_cell)

			path_blocked = 0
			while (path_blocked == 0) and (len(cell_path)<self.max_traj_length):
				#calculate next movement
				mov_direction = self.calculate_next_movement(current_cell,mov_direction)
				mov_path.append(mov_direction)

				if mov_direction != -1:
					#calculate destination cell
					current_cell = self.calculate_destination_cell(current_cell,mov_direction)				

					if current_cell != -1: # valid cell
						cell_path.append(current_cell)
					else: # cell out of bounds
						path_blocked = 1 
						
				else:
					path_blocked = 1
		



		# 3 - compute the trajectory path backwards until the ini point given by the stefmap prediction
		mov_direction = int(self.stefmap_prediction["stefmap"]["cells"][starting_cell]["best_angle"]/45)

		current_cell = starting_cell
		max_path_backwards = self.max_traj_length - len(cell_path)
		path_len = 0
		while current_cell != -1 and path_len < max_path_backwards:
		#for i in range(0,100):
			current_cell,mov_direction = self.calculate_backwards_cell(current_cell,mov_direction)
			if current_cell != -1:
				backwards_cell_path.append(current_cell)
			path_len = path_len + 1

		return cell_path,backwards_cell_path

	def calculate_destination_cell(self,current_cell,direction_of_movement):
		current_row = self.stefmap_prediction["stefmap"]["cells"][current_cell]["row"]
		current_col = self.stefmap_prediction["stefmap"]["cells"][current_cell]["column"]

		if direction_of_movement == 0:
			destination_row = current_row + 1
			destination_col = current_col 
		elif direction_of_movement == 1:
			destination_row = current_row + 1
			destination_col = current_col + 1	
		elif direction_of_movement == 2:
			destination_row = current_row 
			destination_col = current_col + 1		
		elif direction_of_movement == 3:
			destination_row = current_row -1
			destination_col = current_col + 1		
		elif direction_of_movement == 4:
			destination_row = current_row -1
			destination_col = current_col 		
		elif direction_of_movement == 5:
			destination_row = current_row - 1 
			destination_col = current_col - 1		
		elif direction_of_movement == 6:
			destination_row = current_row 
			destination_col = current_col -1		
		elif direction_of_movement == 7:
			destination_row = current_row + 1
			destination_col = current_col - 1

		destination_cell = destination_col + destination_row*self.width
		#transform row and col into cell index
		if destination_row < 0 or destination_row > self.width or destination_col < 0 or destination_col > self.height or sum(self.stefmap_prediction["stefmap"]["cells"][destination_cell]["probabilities"])==0:
			destination_cell = -1 #cell out of bounds
			
		return destination_cell

	def calculate_backwards_cell(self,current_cell,direction_of_movement):
		if direction_of_movement == 0:
			forbidden_coming_orientations = [1,0,7]
		elif direction_of_movement == 1:
			forbidden_coming_orientations = [2,1,0]
		elif direction_of_movement == 2:
			forbidden_coming_orientations = [3,2,1]
		elif direction_of_movement == 3:
			forbidden_coming_orientations = [4,3,2]
		elif direction_of_movement == 4:
			forbidden_coming_orientations = [5,4,3]
		elif direction_of_movement == 5:
			forbidden_coming_orientations = [6,5,4]
		elif direction_of_movement == 6:
			forbidden_coming_orientations = [7,6,5]
		elif direction_of_movement == 7:
			forbidden_coming_orientations = [0,7,6]

		#check the allowed cells of procedence, which is the prob high value of jumping into the current cell
		if self.use_montercarlo == "false":
			max_prob = 0
			for o in range(0,8):
				if (o in forbidden_coming_orientations) == False:
					if o == 0:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							prob = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][4]
							if prob > max_prob:
								max_prob = prob
								procedence_cell = origin_cell
								new_mov_direction = 4
					elif o == 1:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							prob = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][5]
							if prob > max_prob:
								max_prob = prob
								procedence_cell = origin_cell
								new_mov_direction = 5
					elif o == 2:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							prob = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][6]
							if prob > max_prob:
								max_prob = prob
								procedence_cell = origin_cell	
								new_mov_direction = 6
					elif o == 3:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							prob = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][7]
							if prob > max_prob:
								max_prob = prob
								procedence_cell = origin_cell
								new_mov_direction = 7
					elif o == 4:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							prob = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][0]
							if prob > max_prob:
								max_prob = prob
								procedence_cell = origin_cell
								new_mov_direction = 0
					elif o == 5:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							prob = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][1]
							if prob > max_prob:
								max_prob = prob
								procedence_cell = origin_cell
								new_mov_direction = 1
					elif o == 6:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							prob = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][2]
							if prob > max_prob:
								max_prob = prob
								procedence_cell = origin_cell
								new_mov_direction = 2
					elif o == 7:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							prob = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][3]
							if prob > max_prob:
								max_prob = prob
								procedence_cell = origin_cell
								new_mov_direction = 3

			if max_prob == 0:
				procedence_cell = -1
				new_mov_direction = -1 

		else:
			probabilities = [0,0,0,0,0,0,0,0]
			origin_cells =  [0,0,0,0,0,0,0,0]
			for o in range(0,8):
				if (o in forbidden_coming_orientations) == False:
					if o == 0:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							probabilities[4] = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][4]
							origin_cells[4] = origin_cell
					elif o == 1:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							probabilities[5] = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][5]
							origin_cells[5] = origin_cell
					elif o == 2:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							probabilities[6] = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][6]
							origin_cells[6] = origin_cell
					elif o == 3:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							probabilities[7] = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][7]
							origin_cells[7] = origin_cell
					elif o == 4:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							probabilities[0] = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][0]
							origin_cells[0] = origin_cell
					elif o == 5:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							probabilities[1] = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][1]
							origin_cells[1] = origin_cell
					elif o == 6:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							probabilities[2] = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][2]
							origin_cells[2] = origin_cell
					elif o == 7:
						origin_cell = self.calculate_destination_cell(current_cell,o)
						if origin_cell != -1:
							probabilities[3] = self.stefmap_prediction["stefmap"]["cells"][origin_cell]["probabilities"][3]
							origin_cells[3] = origin_cell

			sum_prob = sum(probabilities)
			if sum_prob == 0:
				new_mov_direction = -1
				procedence_cell = -1
			
			else:
				# normalise probabilities over 1
				for o in range (0,8):
					probabilities[o] = probabilities[o]/sum_prob 
				
				new_mov_direction = np.random.choice([0,1,2,3,4,5,6,7],p=probabilities)
				procedence_cell = origin_cells[new_mov_direction]


		return procedence_cell,new_mov_direction

	def calculate_next_movement(self, current_cell,last_mov_direction):
		if last_mov_direction == 0:
			forbidden_orientations = [3,4,5]
		elif last_mov_direction == 1:
			forbidden_orientations = [4,5,6]
		elif last_mov_direction == 2:
			forbidden_orientations = [5,6,7]
		elif last_mov_direction == 3:
			forbidden_orientations = [6,7,0]
		elif last_mov_direction == 4:
			forbidden_orientations = [7,0,1]
		elif last_mov_direction == 5:
			forbidden_orientations = [0,1,2]
		elif last_mov_direction == 6:
			forbidden_orientations = [1,2,3]
		elif last_mov_direction == 7:
			forbidden_orientations = [2,3,4]

		# find the next traj step
		if self.use_montercarlo == "false":
			max_prob = 0
			for o in range(0,8):
				if (o in forbidden_orientations) == False:
					orientation_prob = self.stefmap_prediction["stefmap"]["cells"][current_cell]["probabilities"][o]
					if orientation_prob > max_prob:
						max_prob = orientation_prob
						mov_direction = o

			if max_prob == 0: #all the allowed movement report as not 0 probability so the trajectorey ends
				mov_direction = -1

		else:
			probabilities = [0,0,0,0,0,0,0,0]
			for o in range (0,8):
				if (o in forbidden_orientations) == False:
					probabilities[o] = self.stefmap_prediction["stefmap"]["cells"][current_cell]["probabilities"][o]

			sum_prob = sum(probabilities)

			if sum_prob == 0:
				mov_direction = -1
			
			else:
				# normalise probabilities over 1
				for o in range (0,8):
					probabilities[o] = probabilities[o]/sum_prob 
				
				mov_direction = np.random.choice([0,1,2,3,4,5,6,7],p=probabilities)


		return mov_direction

	def show_trajectory_rviz(self):
		#marker for the starting point
		trajectory_marker = Marker()
		trajectory_marker.header.frame_id = self.frame_id
		trajectory_marker.header.stamp = rospy.get_rostime()
		trajectory_marker.type = 6 #cube list
		trajectory_marker.id = 0
		trajectory_marker.action = 0 # add
		#size
		trajectory_marker.scale.x = 1
		trajectory_marker.scale.y = 1
		trajectory_marker.scale.z = 0.1
		#color
		trajectory_marker.color.r = 1
		trajectory_marker.color.g = 1
		trajectory_marker.color.b = 0
		trajectory_marker.color.a = 1
		
		traj_point = Point()
		traj_point.x = self.stefmap_prediction["stefmap"]["cells"][self.trajectory_cell_path[0]]["x"]
		traj_point.y = self.stefmap_prediction["stefmap"]["cells"][self.trajectory_cell_path[0]]["y"]
		traj_point.z = -0.1
		trajectory_marker.points.append(traj_point)
		self.markerPub.publish(trajectory_marker)



		#marker for the forward of the trajectory
		trajectory_marker = Marker()
		trajectory_marker.header.frame_id = self.frame_id
		trajectory_marker.header.stamp = rospy.get_rostime()
		trajectory_marker.type = 6 #cube list
		trajectory_marker.id = 1
		trajectory_marker.action = 0 # add
		#size
		trajectory_marker.scale.x = 1
		trajectory_marker.scale.y = 1
		trajectory_marker.scale.z = 0.1
		#color
		trajectory_marker.color.r = 1
		trajectory_marker.color.g = 1
		trajectory_marker.color.b = 1
		trajectory_marker.color.a = 1

		#points
		for c in range(1,len(self.trajectory_cell_path)):
			traj_point = Point()
			traj_point.x = self.stefmap_prediction["stefmap"]["cells"][self.trajectory_cell_path[c]]["x"]
			traj_point.y = self.stefmap_prediction["stefmap"]["cells"][self.trajectory_cell_path[c]]["y"]
			traj_point.z = -0.1
			trajectory_marker.points.append(traj_point)

		self.markerPub.publish(trajectory_marker)

		trajectory_marker = Marker()


		#marker for the backward of the trajectory
		trajectory_marker = Marker()
		trajectory_marker.header.frame_id = self.frame_id
		trajectory_marker.header.stamp = rospy.get_rostime()
		trajectory_marker.type = 6 #cube list
		trajectory_marker.id = 2
		trajectory_marker.action = 0 # add
		#size
		trajectory_marker.scale.x = 1
		trajectory_marker.scale.y = 1
		trajectory_marker.scale.z = 0.1
		#color
		trajectory_marker.color.r = 0
		trajectory_marker.color.g = 1
		trajectory_marker.color.b = 1
		trajectory_marker.color.a = 1

		#points
		for c in range(0,len(self.backwards_cell_path)):
			traj_point = Point()
			traj_point.x = self.stefmap_prediction["stefmap"]["cells"][self.backwards_cell_path[c]]["x"]
			traj_point.y = self.stefmap_prediction["stefmap"]["cells"][self.backwards_cell_path[c]]["y"]
			traj_point.z = -0.1
			trajectory_marker.points.append(traj_point)

		self.markerPub.publish(trajectory_marker)

		trajectory_marker = Marker()

	#def run(self):
	#	r = rospy.Rate(0.2)
	#	while not rospy.is_shutdown():
	#		self.trajectory_cell_path,self.backwards_cell_path = self.compute_trajectory()
	#		print "trajectory cell path",self.trajectory_cell_path
	#		print "backwards cell path",self.backwards_cell_path
	#		self.show_trajectory_rviz()
	#		r.sleep()

	def run(self):
		trajectories_generated = 0
		while trajectories_generated < self.num_traj_to_generate and not rospy.is_shutdown():
			self.trajectory_cell_path,self.backwards_cell_path = self.compute_trajectory()
			full_path = self.backwards_cell_path[::-1]+self.trajectory_cell_path
			print full_path
			path_len = len(full_path)
			if path_len > self.min_traj_length: #save the path 
				x_path=[]
				y_path=[]
				for i in range(0,path_len):
					x_path.append(self.stefmap_prediction["stefmap"]["cells"][full_path[i]]["x"])
					y_path.append(self.stefmap_prediction["stefmap"]["cells"][full_path[i]]["y"])

				#append in the file

				data_to_write = {"x":x_path,"y":y_path}
				with open(self.output_filename,"a") as ofile:
					json.dump(data_to_write,ofile)
					ofile.write("\n")
				trajectories_generated = trajectories_generated + 1

			else:
				print "path too short"
			#self.show_trajectory_rviz()
			rospy.sleep(0.5)


if __name__ == '__main__':
	rospy.init_node('trajectory_generator_node', anonymous=True)
	tgn = trajectory_generator_node()

