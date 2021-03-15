#!/usr/bin/env python3

import roslib
import rospy
import actionlib
from std_msgs.msg import String
from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal

class IdleState:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('idle_state', PresentAction, self.execute, False)
        self.server.start()
    
    def execute(self, goal):
        # Send goal to phone
        # Send goal to voice module
        self.server.set_succeeded()

rospy.init_node("idle_state_server")
server = IdleState()
rospy.spin()