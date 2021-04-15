#!/usr/bin/env python3

import roslib
import rospy
import actionlib
from std_msgs.msg import String
from tts.msg import SpeechAction, SpeechGoal
from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal

class PresentState:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('present_state', PresentAction, self.execute, False)
        self.server.start()
        self.publisher = rospy.Publisher('present', String, queue_size=1)
        self.voice_client = actionlib.SimpleActionClient('tts', SpeechAction)
        self.voice_client.wait_for_server()
    
    def execute(self, goal):
        # Send goal to phone
        # Send goal to voice module
        self.publisher.publish(goal.text)
        voice_command = SpeechGoal()
        voice_command.text = goal.text
        self.voice_client.send_goal(voice_command)
        self.voice_client.wait_for_result()
        self.server.set_succeeded()

rospy.init_node("present_state_server")
server = PresentState()
rospy.spin()