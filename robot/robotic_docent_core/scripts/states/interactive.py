#!/usr/bin/env python3

import roslib
import rospy
import actionlib
from std_msgs.msg import String
import time
import queue
from robotic_docent_core.msg import Piece, QAAction, QAGoal, MotionAction, MotionGoal, PresentAction, PresentGoal


class InteractiveState:
    def __init__(self, timeout):
        self.interactive_server = actionlib.SimpleActionServer('interactive_state', PresentAction, self.execute, False)
        self.queue = queue.Queue()
        self.queue_server = actionlib.SimpleActionServer('queue_server', QAAction, self.add_to_queue, False)
        self.interactive_server.start()
        self.queue_server.start()
        # timeout parameter is to let museum operators decide how
        # long will they let the robot idle in this phase
        self.timeout = timeout
    
    def execute(self, goal):
        # in progress
        t = self.timeout
        ac = actionlib.SimpleActionClient('present_state', PresentAction)
        while t:
            if len(self.queue.queue) == 0:
                mins, secs = divmod(t, 60) 
                timer = '{:02d}:{:02d}'.format(mins, secs)  
                time.sleep(1)
                t -= 1
            else:
                t = self.timeout
                current_qa = self.queue.get()
                current_interaction = PresentGoal()
                current_interaction.text = ''.join(["You asked the question ", current_qa.question])
                ac.send_goal(current_interaction)
                ac.wait_for_result()
                current_interaction.text = ''.join(["The answer is ", current_qa.answer])
                ac.send_goal(current_interaction)
                ac.wait_for_result()
                    
        self.interactive_server.set_succeeded()
    
    def add_to_queue(self, goal):
        self.queue.put(goal)
        self.queue_server.set_succeeded()

rospy.init_node("interactive_state_server")
server = InteractiveState(60)
rospy.spin()