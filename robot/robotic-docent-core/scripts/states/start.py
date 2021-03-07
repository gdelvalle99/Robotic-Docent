#!/usr/bin/env python

import roslib
roslib.load_manifest('robotic-docent-core')
import rospy
import actionlib
from std_msgs import String

class StartState:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('start_state', String, self.execute, False)
        self.server.start()
        self.publisher = rospy.Publisher('start', String)
    
    def execute(self, goal):
        # Send goal to phone
        # Send goal to voice module
        self.publisher.publish(goal)
        self.server.set_succeeded()

rospy.init_node("start_state_server")
server = StartState()
rospy.spin()