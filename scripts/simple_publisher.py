#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Float32

def talker():
  pub = rospy.Publisher('my_random_float', Float32)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(22)

  while not rospy.is_shutdown():
    random_number = random.random()*10
    rospy.loginfo(random_number)
    pub.publish(random_number)
    rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
