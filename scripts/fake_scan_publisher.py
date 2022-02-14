#!/usr/bin/env python
import rospy
import random
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
def talker():
  rospy.init_node('fake_scan_publisher', anonymous=True)
  
  publish_topic = rospy.get_param("publish_topic", "fake_scan")
  publish_rate = rospy.get_param("publish_rate", 20)
  rate = rospy.Rate(publish_rate)
  angle_min = rospy.get_param("angle_min",(-2.0/3.0)*math.pi)
  angle_max = rospy.get_param("angle_max",(2.0/3.0)*math.pi)
  angle_increment = rospy.get_param("angle_increment",(1.0/300.0)*math.pi)
  range_min = rospy.get_param("range_min",1.0)
  range_max = rospy.get_param("range_max",10.0)
   
  pub = rospy.Publisher(publish_topic, LaserScan, queue_size=10)
  while not rospy.is_shutdown():
    header = Header(0,rospy.get_rostime(),"base_link")
    frame_id = "base_link"

    cur_angle = angle_min
    ranges = []
    for i in range(401):
      ranges.append(random.uniform(range_min, range_max))
      cur_angle += angle_increment
 
    ls = LaserScan(header=header,
                angle_min=angle_min,
                angle_max=angle_max,
                angle_increment=angle_increment,
                range_min=range_min,
                range_max=range_max,
                ranges=ranges)
    #rospy.loginfo(ls)
    pub.publish(ls)
    rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
