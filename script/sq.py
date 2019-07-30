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
    n = 0
    vel_msg.linear.x = 1; 
    while not rospy.is_shutdown():
        n += 1
        key = cv2.waitKey(1)
        rospy.loginfo(n)
	if key == 113:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0 
          rospy.loginfo('esq')
          pub.publish(vel_msg)
          break
        if n > 20:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 1
          rospy.loginfo('turn')
        if n > 29:
          vel_msg.linear.x = 1
          vel_msg.angular.z = 0
          rospy.loginfo('go')
          n = 0
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

