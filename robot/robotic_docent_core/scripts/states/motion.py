#!/usr/bin/env python3

import roslib
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal

class MotionState:
    """
    Robot state implemented by actionlib. 
    Intermediary between robot and map navigation module.
    """
    def __init__(self):
        self.server = actionlib.SimpleActionServer('move_state', MotionAction, self.execute, False)
        self.server.start()
        self.motionstate_publisher = rospy.Publisher("motion_state_publisher", String, queue_size=1)
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    def execute(self, goal):
        ac = actionlib.SimpleActionClient("map_navigation", MotionAction)
        ac.wait_for_server()
        self.motionstate_publisher.publish("Sending " + str(goal) + " to map_navigation")
        ac.send_goal(goal)
        ac.wait_for_result()
        if ac.get_state() == 1:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

        # self.navigate(goal)

    def convert_array_to_point(arr):
        return Point(arr[0], arr[1])
    
    def navigate(self, nav):
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        # self.map_navigation_publisher.publish("In map navigation")
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.motionstate_publisher.publish("Connecting to move base")
        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")


        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal

        goal.target_pose.pose.position = Point(nav.goal_x, nav.goal_y, 0)   
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        self.motionstate_publisher.publish("Sending goal to move base")
        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)
        ac.wait_for_result()
        if ac.get_state() == actionlib.SimpleGoalState.SUCCEEDED:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

rospy.init_node("motion_state_server")
server = MotionState()
rospy.spin()