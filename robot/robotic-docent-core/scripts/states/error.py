#!/usr/bin/env python

import roslib
roslib.load_manifest('robotic-docent-core')
import rospy
import actionlib
from std_msgs import String

class ErrorState:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('error_state', String, self.execute, False)
        self.server.start()
    
    def execute(self, goal):
        # Send goal to phone
        # Send goal to voice module
        self.server.set_succeeded()

rospy.init_node("error_state_server")
server = ErrorState()
rospy.spin()