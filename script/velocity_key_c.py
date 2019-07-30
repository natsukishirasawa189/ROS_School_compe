#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib
import cv2
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    vel_msg = Twist()
    while not rospy.is_shutdown():
        key = cv2.waitKey(1)
        rospy.loginfo(key)
	if key == 115:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0 
        if key == 119:
	  vel_msg.linear.x += 0.1
        if key == 120:
	  vel_msg.linear.x += -0.1
        if key == 97:
          vel_msg.angular.z += 0.1
        if key == 100:
          vel_msg.angular.z += -0.1
        if key == 113:
          break
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
  rospy.init_node('velocity_key_c')
  image = cv2.imread(roslib.packages.get_pkg_dir('lecture_pkg') + '/files/InputKey.png')
  cv2.imshow('InputWindow',image)
  try:
      talker()
  except rospy.ROSInterruptException:
      pass

