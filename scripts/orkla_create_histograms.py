import rospy
import sys
import time
import datetime
import math
import os
import numpy as np



if __name__ == '__main__':

	# Map dimensions and cell size
	xmin = -10 # meters
	xmax = 60   # meters
	ymin = -10 # meters
	ymax = 60 # meters
	cell_size = 1 #meters
	num_bins = 8

	# starting and ending time, time interval.
	time_interval = 3600 #seconds
	
	# DAY 1 - COMPUTE HISTOGRAMS
	input_file_name =  "./../data/Orkla_2019-06-13_people_detections.txt"
	output_file_name = "./../data/orkla_2019_06_13_histograms.txt"
	t_ini = 1560384000 #13 june 2019
	t_end = 1560384000 + 86400

	print("Creating flowmap histograms...")
	os.system("python ./tools/create_stefmap_input_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(num_bins)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")

	# DAY 2 - COMPUTE HISTOGRAMS
	input_file_name =  "./../data/Orkla_2019-06-14_people_detections.txt"
	output_file_name = "./../data/orkla_2019_06_14_histograms.txt"
	t_ini = 1560470400 #14 june 2019
	t_end = 1560470400 + 86400

	print("Creating flowmap histograms...")
	os.system("python ./tools/create_stefmap_input_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(num_bins)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")

	# DAY 3 - COMPUTE HISTOGRAMS
	input_file_name =  "./../data/Orkla_2019-06-18_people_detections.txt"
	output_file_name = "./../data/orkla_2019_06_18_histograms.txt"
	t_ini = 1560816000 #18 june 2019
	t_end = 1560816000 + 86400

	print("Creating flowmap histograms...")
	os.system("python ./tools/create_stefmap_input_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(num_bins)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done") 
	
	# DAY 4 - COMPUTE HISTOGRAMS
	input_file_name =  "./../data/Orkla_2019-06-19_people_detections.txt"
	output_file_name = "./../data/orkla_2019_06_19_histograms.txt"
	t_ini = 1560902400 #19 june 2019
	t_end = 1560902400 + 86400

	print("Creating flowmap histograms...")
	os.system("python ./tools/create_stefmap_input_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(num_bins)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")