#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import Tkinter

class map_navigation:
  def __init__(self, query):
    # initiliaze
    self.query = query
    self.queryIndex = 0
    # rospy.init_node('map_navigation', anonymous=False)



  def shutdown(self):
      rospy.loginfo("Quit program")
      rospy.sleep()

  def previousGoal(self, root, text):
      if(self.queryIndex == 0):
        text.delete("1.0", Tkinter.END)
        text.insert(Tkinter.END, "No previous exhibit!")
        return
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")
      
      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/

      goal.target_pose.pose.position =  Point(self.query[self.queryIndex]['coordinates']['x'],self.query[self.queryIndex]['coordinates']['y'],0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)

      text.delete("1.0", Tkinter.END)
      text.insert(Tkinter.END, "Waiting")
      ac.wait_for_result(rospy.Duration(60))
      text.delete("1.0", Tkinter.END)
      text.insert(Tkinter.END, self.query[self.queryIndex]['script'])
      self.queryIndex -= 1

  def nextGoal(self, root, text):
      if(self.queryIndex == len(self.query)-1):
        text.delete("1.0", Tkinter.END)
        text.insert(Tkinter.END, "No next exhibit!")
        return
      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
      
      #wait for the action server to come up
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")


      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal

      goal.target_pose.pose.position =  Point(self.query[self.queryIndex]['coordinates']['x'],self.query[self.queryIndex]['coordinates']['y'],0)      
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)
      text.delete("1.0", Tkinter.END)
      text.insert(Tkinter.END, "Waiting")
      ac.wait_for_result(rospy.Duration(60))
      text.delete("1.0", Tkinter.END)
      text.insert(Tkinter.END, self.query[self.queryIndex]['script'])
      self.queryIndex += 1

