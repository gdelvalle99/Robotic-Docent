#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from robotic_docent_core.msg import Piece, MotionAction, MotionGoal,  PresentAction, PresentGoal
from std_msgs.msg import String

class map_navigation:
  def __init__(self):
    # initiliaze
    self.server = actionlib.SimpleActionServer("map_navigation", MotionAction, self.navigate, False)
    self.map_navigation_publisher = rospy.Publisher("map_navigation_publisher", String, queue_size=1)
    self.server.start()

  def navigate(self, nav):
      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      self.map_navigation_publisher.publish("In map navigation")
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
      self.map_navigation_publisher.publish("Connecting to move base")
      #wait for the action server to come up
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")


      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal

      goal.target_pose.pose.position = Point(nav.goal_x, nav.goal_y, 0)   
      goal.target_pose.pose.orientation.w = 1.0
      self.map_navigation_publisher.publish("Sending goal to move base")
      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)
      ac.wait_for_result()
      if ac.get_state() == 4:
        self.server.set_aborted()
      else:
        self.server.set_succeeded()

rospy.init_node('map_navigation', anonymous=False)
server = map_navigation()
rospy.spin()