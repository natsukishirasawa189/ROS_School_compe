#!/usr/bin/env python

import rospy
import actionlib
from lecture_pkg.msg import *

class DoDishesServer:

  def __init__(self):
    self.server = actionlib.SimpleActionServer('do_dishes',DoDishesAction,self.execute,False)
    self.server.start()

  def execute(self,goal):
    print "Requestting dishwasher %d"%(goal.dishwasher_id)
    result = self.server.get_default_result()
    result.total_dishes_cleaned = 100
    feedback = DoDishesFeedback()
    for i in xrange(result.total_dishes_cleaned):
      feedback.percent_complete = i+1
      print "Percent of dishes_cleaned %d"%(feedback.percent_complete) + "%"
      self.server.publish_feedback(feedback)

    print "Returning dishes_cleaned %d"%(result.total_dishes_cleaned)
    self.server.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('do_dishes_server')
  server = DoDishesServer()
  rospy.spin()
