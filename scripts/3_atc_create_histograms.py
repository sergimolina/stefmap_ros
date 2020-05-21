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
	xmin = -60 # meters
	xmax = 80   # meters
	ymin = -40 # meters
	ymax = 20 # meters
	cell_size = 1 #meters

	# starting and ending time, time interval.
	time_interval = 3600 #seconds

	base_dir_i = "/home/ksatyaki/workspace/DATA/ATC/part1_1s/"
	base_dir_o = "/home/ksatyaki/workspace/cpp_ws/src/mod_ros/stefmap_ros/data/"

	input_file_name = base_dir_i + "20121024_ds_1s.csv"
	output_file_name = base_dir_o + "atc_2012_10_24_histograms.txt"
	t_ini = 1351036800 # 24th October 2012
	t_end = 1351036800 + 86400

	print("Creating flowmap histograms for day 1...")
	os.system("python ./tools/create_flowmap_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")

	input_file_name = base_dir_i + "20121028_ds_1s.csv"
	output_file_name = base_dir_o + "atc_2012_10_28_histograms.txt"
	t_ini = 1351382400  # 28th October 2012
	t_end = 1351382400 + 86400

	print("Creating flowmap histograms for day 2...")
	os.system("python ./tools/create_flowmap_histograms.py "+input_file_name+" "+ output_file_name+" "+str(xmin)+" "+str(xmax)+" "+str(ymin)+" "+str(ymax)+" "+str(cell_size)+" "+str(t_ini)+" "+str(t_end)+" "+str(time_interval))
	print("Done")

	input_file_name = base_dir_i + "20121031_ds_1s.csv"
	output_file_name = base_dir_o + "atc_2012_10_31_histograms.txt"
	t_ini = 1351641600  # 31st October 2012
	t_end = 1351641600 + 86400

	print("Creating flowmap histograms for day 3...")
	os.system("python ./tools/create_flowmap_histograms.py " + input_file_name + " " + output_file_name + " " + str(
		xmin) + " " + str(xmax) + " " + str(ymin) + " " + str(ymax) + " " + str(cell_size) + " " + str(t_ini) + " " + str(
		t_end) + " " + str(time_interval))
	print("Done")

	input_file_name = base_dir_i + "20121104_ds_1s.csv"
	output_file_name = base_dir_o + "atc_2012_11_04_histograms.txt"
	t_ini = 1351987200  # 04th November 2012
	t_end = 1351987200 + 86400

	print("Creating flowmap histograms for day 4...")
	os.system("python ./tools/create_flowmap_histograms.py " + input_file_name + " " + output_file_name + " " + str(
		xmin) + " " + str(xmax) + " " + str(ymin) + " " + str(ymax) + " " + str(cell_size) + " " + str(t_ini) + " " + str(
		t_end) + " " + str(time_interval))
	print("Done")

	input_file_name = base_dir_i + "20121107_ds_1s.csv"
	output_file_name = base_dir_o + "atc_2012_11_07_histograms.txt"
	t_ini = 1352246400  # 07th November 2012
	t_end = 1352246400 + 86400

	print("Creating flowmap histograms for day 5...")
	os.system("python ./tools/create_flowmap_histograms.py " + input_file_name + " " + output_file_name + " " + str(
		xmin) + " " + str(xmax) + " " + str(ymin) + " " + str(ymax) + " " + str(cell_size) + " " + str(t_ini) + " " + str(
		t_end) + " " + str(time_interval))
	print("Done")

	input_file_name = base_dir_i + "20121111_ds_1s.csv"
	output_file_name = base_dir_o + "atc_2012_11_11_histograms.txt"
	t_ini = 1352592000  # 11th November 2012
	t_end = 1352592000 + 86400

	print("Creating flowmap histograms for day 6...")
	os.system("python ./tools/create_flowmap_histograms.py " + input_file_name + " " + output_file_name + " " + str(
		xmin) + " " + str(xmax) + " " + str(ymin) + " " + str(ymax) + " " + str(cell_size) + " " + str(t_ini) + " " + str(
		t_end) + " " + str(time_interval))
	print("Done")

