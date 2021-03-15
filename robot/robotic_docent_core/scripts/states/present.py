#!/usr/bin/env python3

import roslib
import rospy
import actionlib
from std_msgs.msg import String
from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal

class PresentState:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('present_state', PresentAction, self.execute, False)
        self.server.start()
        self.publisher = rospy.Publisher('present', String, queue_size=1)
    
    def execute(self, goal):
        # Send goal to phone
        # Send goal to voice module
        self.publisher.publish(goal)
        self.server.set_succeeded()

rospy.init_node("present_state_server")
server = PresentState()
rospy.spin()