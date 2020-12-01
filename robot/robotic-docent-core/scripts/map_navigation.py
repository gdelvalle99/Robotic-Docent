#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point


class map_navigation:
  def choose(self):
    choice='q'
    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|PRESSE A KEY:")
    rospy.loginfo("|'0' ")
    rospy.loginfo("|'1' ")
    rospy.loginfo("|'2' ")
    rospy.loginfo("|'3' ")
    rospy.loginfo("|'q': Quit ")
    rospy.loginfo("|-------------------------------|")
    choice = input()
    return choice

  def __init__(self):
    # declare the coordinates of interest
    self.xLoc1 = 12
    self.yLoc1 = 26
    self.xLoc2 = 10
    self.yLoc2 = 13
    self.xLoc3 = 41
    self.yLoc3 = 17
    self.xLoc4 = 48
    self.yLoc4 = 41
    self.goalReached = False
    # initiliaze
    rospy.init_node('map_navigation', anonymous=False)
    rospy.loginfo("Choose a x coordinate")
    choiceX = input()
    rospy.loginfo("Choose a y coordinate")
    choiceY = input()
    self.goalReached = self.moveToGoal(choiceX, choiceY)

    while choiceX != '':
      rospy.loginfo("Choose a x coordinate")
      choiceX = input()
      rospy.loginfo("Choose a y coordinate")
      choiceY = input()
      self.goalReached = self.moveToGoal(choiceX, choiceY)

      if (choiceX!=''):
        if (self.goalReached):
          rospy.loginfo("Congratulations!")
          #rospy.spin()

        else:
          rospy.loginfo("Hard Luck!")


  def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

  def moveToGoal(self,xGoal,yGoal):
      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

      #wait for the action server to come up
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")


      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/

      goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)

      ac.wait_for_result(rospy.Duration(60))

      if(ac.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.loginfo("You have reached the destination")
              return True

      else:
              rospy.loginfo("The robot failed to reach the destination")
              return False

if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")