#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class laserscan_coverage_filter(object):

  def __init__(self):
    #parameters
    self.scan_1_topic = rospy.get_param('~scan_1_topic',"/scan_1")
    self.scan_2_topic = rospy.get_param('~scan_2_topic',"/scan_2")

    # subscribe to topics
    rospy.Subscriber(self.scan_1_topic, LaserScan, self.scan_1_callback, queue_size=1)
    rospy.Subscriber(self.scan_2_topic, LaserScan, self.scan_2_callback, queue_size=1)

    # create topic publishers
    self.scan_filtered_pub = rospy.Publisher(self.scan_1_topic+"/filtered", LaserScan, queue_size=1)

    self.run()

  def scan_1_callback(self,scan_msg):
    scan_1 = scan_msg

    scan_filtered_msg = LaserScan()
    scan_filtered_msg.header = scan_1.header
    scan_filtered_msg.angle_min = scan_1.angle_min
    scan_filtered_msg.angle_max = scan_1.angle_max
    scan_filtered_msg.angle_increment = scan_1.angle_increment
    scan_filtered_msg.time_increment = scan_1.time_increment
    scan_filtered_msg.scan_time = scan_1.scan_time
    scan_filtered_msg.range_min = scan_1.range_min
    scan_filtered_msg.range_max = scan_1.range_max

    for i in range(0,len(scan_1.ranges)):
      if np.isinf(scan_1.ranges[i]):
        if np.isinf(self.scan_2.ranges[i]):
          scan_filtered_msg.ranges.append(0)
        else:
          scan_filtered_msg.ranges.append(scan_1.range_max)

      else:
        scan_filtered_msg.ranges.append(scan_1.ranges[i])

    self.scan_filtered_pub.publish(scan_filtered_msg)


  def scan_2_callback(self,scan_msg):
    self.scan_2 = scan_msg


  def run(self):        
    while not rospy.is_shutdown():
      rospy.spin()


if __name__ == '__main__':
  rospy.init_node('laserscan_coverage_filter_node', anonymous=True)
  lcf = laserscan_coverage_filter()
