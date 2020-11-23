#!/usr/bin/python2

import os

if __name__ == '__main__':
	base_dir_o = "/home/ksatyaki/workspace/cpp_ws/src/mod_ros/stefmap_ros/data/"

	output_file_name = base_dir_o + "atc_2012_10_24_histograms.txt"
	print("Loading day 1 histograms to FreMEn...")
	os.system("rosrun stefmap_ros load_histograms.py "+output_file_name)
	print("Done")

	output_file_name = base_dir_o + "atc_2012_10_28_histograms.txt"
	print("Loading day 2 histograms to FreMEn...")
	os.system("rosrun stefmap_ros load_histograms.py "+output_file_name)
	print("Done")

	output_file_name = base_dir_o + "atc_2012_10_31_histograms.txt"
	print("Loading day 3 histograms to FreMEn...")
	os.system("rosrun stefmap_ros load_histograms.py " + output_file_name)
	print("Done")

	output_file_name = base_dir_o + "atc_2012_11_04_histograms.txt"
	print("Loading day 4 histograms to FreMEn...")
	os.system("rosrun stefmap_ros load_histograms.py " + output_file_name)
	print("Done")

	output_file_name = base_dir_o + "atc_2012_11_07_histograms.txt"
	print("Loading day 5 histograms to FreMEn...")
	os.system("rosrun stefmap_ros load_histograms.py " + output_file_name)
	print("Done")

	output_file_name = base_dir_o + "atc_2012_11_11_histograms.txt"
	print("Loading day 6 histograms to FreMEn...")
	os.system("rosrun stefmap_ros load_histograms.py " + output_file_name)
	print("Done")

