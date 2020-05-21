#!/usr/bin/python2
import os

if __name__ == '__main__':
    #python ~/workspace/cpp_ws/src/mod_ros/stefmap_ros/scripts/tools/load_histograms.py ../histograms.txt
    output_file_name = "/home/ksatyaki/workspace/DATA/UTBM/histograms.txt"
    print("Loading histograms to FreMEn...")
    os.system("python ~/workspace/cpp_ws/src/mod_ros/stefmap_ros/scripts/tools/load_histograms.py "+output_file_name)
    print("Done")
