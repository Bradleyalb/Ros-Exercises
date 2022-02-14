#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32

class simple_subscriber:
  def __init__(self):
    rospy.init_node('simple_subscriber', anonymous=True)
    rospy.Subscriber('my_random_float', Float32, self.callback)
    self.pub = rospy.Publisher('random_float_log', Float32, queue_size=10)
  
  def callback(self, data):
    rospy.loginfo(math.log(data.data))
    #print(math.log(data.data))
    self.pub.publish(Float32(math.log(data.data)))
  def listener(self):
    rospy.spin()

if __name__ == '__main__':
  ss = simple_subscriber()
  ss.listener()
