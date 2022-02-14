#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ros_exercises.msg import OpenSpace
class simple_subscriber:
  def __init__(self):
    subscriber_topic = rospy.get_param('subscriber_topic', 'fake_scan')
    publisher_topic = rospy.get_param('publisher_topic', 'open_space')
    
    rospy.init_node('open_space_publisher', anonymous=True)
    rospy.Subscriber(subscriber_topic, LaserScan, self.callback)
    
    self.pub = rospy.Publisher(publisher_topic, OpenSpace, queue_size=10)
  
  def callback(self, data):
    min_angle = data.angle_min
    max_angle = data.angle_max
    angle_increment = data.angle_increment
    max_range = 0
    cur_angle = min_angle
    for r in data.ranges:
      if r > max_range:
        max_range = r
        max_range_angle = cur_angle
      cur_angle+=angle_increment
    os = OpenSpace(max_range_angle, max_range)
    rospy.loginfo(os)
    
    self.pub.publish(os)
  def listener(self):
    rospy.spin()

if __name__ == '__main__':
  ss = simple_subscriber()
  ss.listener()
