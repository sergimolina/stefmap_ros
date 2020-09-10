#!/usr/bin/env python

import rospy
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson, TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseArray,Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class spencermsgs_to_posearray(object):

  def __init__(self):
    #parameters
    self.people_detections_topic = rospy.get_param('~people_detections_topic',"/people_detections")

    # subscribe to topics
    rospy.Subscriber(self.people_detections_topic, DetectedPersons, self.people_detections_callback,queue_size=1)

    # create topic publishers
    self.people_detections_converted_pub = rospy.Publisher(self.people_detections_topic+'/posearray', PoseArray, queue_size=1)

    self.run()

  def people_detections_callback(self,spencer_msg):
    people_detections_msg = PoseArray()
    people_detections_msg.header = spencer_msg.header

    for p in range(0,len(spencer_msg.detections)):
      people_detections_msg.poses.append(spencer_msg.detections[p].pose.pose)
      
    self.people_detections_converted_pub.publish(people_detections_msg)



  def run(self):        
    while not rospy.is_shutdown():
      rospy.spin()


if __name__ == '__main__':
  rospy.init_node('spencermsgs_to_posearray', anonymous=True)
  s2p = spencermsgs_to_posearray()
