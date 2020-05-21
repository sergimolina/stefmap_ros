import rospy
import sys
import time
import datetime
import math
import os
import numpy as np



if __name__ == '__main__':
	
#	output_file_name = "./../data/orkla_2019_10_14_circle_histogram.txt"
#	print("Loading histograms to FreMEn...")
#	os.system("python ./tools/load_histograms.py "+output_file_name)
#	print("Done")


#	output_file_name = "./../data/orkla_2019_10_14_updown_histogram.txt"
#	print("Loading histograms to FreMEn...")
#	os.system("python ./tools/load_histograms.py "+output_file_name)
#	print("Done")

	output_file_name = "./../data/orkla_2019_10_15_scene2_histogram.txt"
	print("Loading histograms to FreMEn...")
	os.system("python ./tools/load_histograms.py "+output_file_name)
	print("Done")