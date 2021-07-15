#!/usr/bin/env python

import rospy
import sys
import time
from std_msgs.msg import String
from fremenarray.msg import FremenArrayActionGoal,FremenArrayActionResult
import numpy as np
import math
from datetime import datetime


if __name__ == '__main__':

	if len(sys.argv) == 12:
		input_file_name = sys.argv[1]
		output_file_name= sys.argv[2]
		# Map dimensions and cell size
		x_min = int(sys.argv[3]) # meters
		x_max = int(sys.argv[4])   # meters
		y_min = int(sys.argv[5]) # meters
		y_max = int(sys.argv[6])   # meters
		grid_size = float(sys.argv[7]) #meters
		num_bins = int(sys.argv[8]) 

		# starting and ending time, time interval.
		t_ini = int(sys.argv[9])   #seconds from 1970
		t_end = int(sys.argv[10])   #seconds from 1970
		time_interval =  int(sys.argv[11])   ##seconds

	else:
		print "This scripts need 11 parameters: input_file_name, output_file_name,xmin,xmax,ymin,ymax,grid_size,num_bins,t_ini,t_end,time_interval "
		sys.exit(1)
	
	# Read the file
	ifile = open(input_file_name,"r")
	ofile = open(output_file_name,"a")

	# change these values depending on the structure of the input file
	input_data = np.genfromtxt(input_file_name, delimiter = ',')

	time_col = 0 
	x_col = 2
	y_col = 3

	# if data is from atc dataset
	theta_col = 6
	input_data[:,x_col] = np.divide(input_data[:,x_col],1000)
	input_data[:,y_col] = np.divide(input_data[:,y_col],1000)

	# start the histogram building
	rows = int((x_max - x_min)/grid_size)
	cols = int((y_max - y_min)/grid_size)
	last_ite = 0;
	bin_counts_matrix = np.zeros([rows,cols,num_bins])

	for t_now in range(t_ini,t_end,time_interval):

		# check people moving to create the histogram
		for i in range(last_ite,len(input_data)):
			if (input_data[i,time_col] >= t_now and input_data[i,time_col] < (t_now+time_interval-0.0001)):
				cell_x = int(math.floor((input_data[i,x_col] - x_min)/grid_size))
				cell_y = int(math.floor((input_data[i,y_col] - y_min)/grid_size))
				if cell_x >= 0 and cell_x < rows and cell_y >= 0 and cell_y < cols:
					if input_data[i,theta_col] < 0:
						angle_bin = math.ceil((math.degrees(input_data[i,theta_col])+360+(180/num_bins))/(360/num_bins)) - 1
						if angle_bin == num_bins:
							angle_bin = 0
					else:
						angle_bin = math.ceil((math.degrees(input_data[i,theta_col])+(180/num_bins))/(360/num_bins)) - 1
					bin_counts_matrix[cell_x][cell_y][int(angle_bin)] = bin_counts_matrix[cell_x][cell_y][int(angle_bin)] + 1

			if (input_data[i,time_col] > (t_now+time_interval-0.0001)):
				last_ite = i; 
				break

		# normalize matrix
		#for r in range(0,rows):
		#	for c in range(0,cols):
		#		max_count = np.amax(bin_counts_matrix[r,c,:])
		#		if max_count > 0:
		#			for b in range(0,num_bins):
		#				bin_counts_matrix[r,c,b] = 100*bin_counts_matrix[r,c,b]/max_count

		# Save  the matrix in the output file
		ofile.write(str(t_now))
		ofile.write(",")
		bin_counts_matrix_1d = np.reshape(bin_counts_matrix,rows*cols*num_bins)
		for j in range(0,rows*cols*num_bins):
			ofile.write(str(int(bin_counts_matrix_1d[j])))
			if j != rows*cols*num_bins-1:
				ofile.write(",")
		ofile.write("\n")

		# Restart matrix
		bin_counts_matrix = np.zeros([rows,cols,num_bins])

		# Verbosity
		print((datetime.utcfromtimestamp(t_now).strftime('%Y-%m-%d %H:%M:%S')))

	ifile.close()
	ofile.close()






