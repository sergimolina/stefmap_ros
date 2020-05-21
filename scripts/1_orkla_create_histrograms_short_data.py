import rospy
import sys
import time
import datetime
import math
import os
import numpy as np



if __name__ == '__main__':

	# Map dimensions and cell size
	xmin = -20 # meters
	xmax = 30   # meters
	ymin = -20 # meters
	ymax = 30 # meters
	cell_size = 1 #meters
	num_bins = 8
	# starting and ending time, time interval.
	time_interval = 86400 #seconds

	

	input_file_name =  "./../data/orkla_2019_10_15_scene2.txt"
	output_file_name = "./../data/orkla_2019_10_15_scene2_histogram_new.txt"
	t_ini = 1571011200 + 86400 #15 october 2019
	t_end = 1571011200 + 86400 + 86400

	print("Creating flowmap histograms...")
	os.system("python ./tools/create_stefmap_input_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(num_bins)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")