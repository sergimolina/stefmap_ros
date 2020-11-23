import rospy
import sys
import time
import datetime
import math
import os
import numpy as np



if __name__ == '__main__':
	
	file_name = "./../data/orkla_2019_06_13_histograms.txt"
	print("Loading histograms to FreMEn...")
	os.system("python ./tools/load_histograms.py "+file_name)
	print("Done")

	file_name = "./../data/orkla_2019_06_14_histograms.txt"
	print("Loading histograms to FreMEn...")
	os.system("python ./tools/load_histograms.py "+file_name)
	print("Done")

	file_name = "./../data/orkla_2019_06_18_histograms.txt"
	print("Loading histograms to FreMEn...")
	os.system("python ./tools/load_histograms.py "+file_name)
	print("Done")

	file_name = "./../data/orkla_2019_06_19_histograms.txt"
	print("Loading histograms to FreMEn...")
	os.system("python ./tools/load_histograms.py "+file_name)
	print("Done")
