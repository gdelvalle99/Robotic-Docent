#!/usr/bin/env python3

import roslib
import rospy
import actionlib
from std_msgs.msg import String
from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal

class ErrorState:
    """
    ROS Node in charge of troubleshooting.
    """
    def __init__(self):
        self.server = actionlib.SimpleActionServer('error_state', PresentAction, self.execute, False)
        self.server.start()
    
    def execute(self, goal):
        # Send goal to phone
        # Send goal to voice module
        self.server.set_succeeded()

rospy.init_node("error_state_server")
server = ErrorState()
rospy.spin()