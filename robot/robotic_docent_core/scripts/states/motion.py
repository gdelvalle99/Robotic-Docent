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
        self.server = actionlib.SimpleActionServer('motion_state', MotionAction, self.execute, False)
        self.server.start()
    
    def execute(self, goal):
        ac = actionlib.SimpleActionClient("map_navigation", MotionAction)
        ac.send_goal_and_wait(self.convert_array_to_point(goal))
        self.server.set_succeeded()

    def convert_array_to_point(arr):
        return Point(arr[0], arr[1])

rospy.init_node("motion_state_server")
server = MotionState()
rospy.spin()