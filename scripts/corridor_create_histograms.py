#!/usr/bin/python2

import rospy
import sys
import time
import datetime
import math
import os
import numpy as np


if __name__ == '__main__':

	# Map dimensions and cell size
	xmin = -25 # meters
	xmax = 25   # meters
	ymin = -25 # meters
	ymax = 25 # meters
	cell_size = 1 #meters
	num_bins = 8

	# starting and ending time, time interval.
	time_interval = 3600 #seconds

	input_file_name = "./../data/corridor_2017_05_31.txt"
	output_file_name = "./../data/corridor_2017_05_31_histograms.txt"
	t_ini = 1496188800 
	t_end = 1496188800 + 86400

	print("Creating flowmap histograms for day 1...")
	os.system("python ./tools/create_stefmap_input_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(num_bins)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")
