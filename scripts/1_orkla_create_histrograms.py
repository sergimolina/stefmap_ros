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
	xmax = 60   # meters
	ymin = -5 # meters
	ymax = 65 # meters
	cell_size = 1 #meters

	# starting and ending time, time interval.
	time_interval = 3600 #seconds
	
	# DAY 1 - CALCULATE HISTOGRAMS AND LOAD THEM INTO FREMEN
	input_file_name =  "./../data/orkla_2019_06_13.txt"
	output_file_name = "./../data/orkla_2019_06_13_histogram.txt"
	t_ini = 1560384000 #13 june 2019
	t_end = 1560384000 + 86400

	print("Creating flowmap histograms...")
	os.system("python ./tools/create_flowmap_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")

	# DAY 2 - CALCULATE HISTOGRAMS AND LOAD THEM INTO FREMEN
	input_file_name =  "./../data/orkla_2019_06_14.txt"
	output_file_name = "./../data/orkla_2019_06_14_histogram.txt"
	t_ini = 1560470400 #14 june 2019
	t_end = 1560470400 + 86400

	print("Creating flowmap histograms...")
	os.system("python ./tools/create_flowmap_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")