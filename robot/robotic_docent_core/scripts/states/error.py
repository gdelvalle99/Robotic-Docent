#!/usr/bin/env python3

import roslib
import rospy
import actionlib
from std_msgs.msg import String
from robotic_docent_core.msg import Piece, ErrorAction, ErrorGoal, PresentAction, PresentGoal, MotionAction, MotionGoal

class ErrorState:
    """
    ROS Node in charge of troubleshooting.
    """
    def __init__(self):
        self.server = actionlib.SimpleActionServer('error_state', ErrorAction, self.execute, False)
        self.server.start()
        self.voice_client = actionlib.SimpleActionClient('present_state', PresentAction)
        self.voice_client.wait_for_server()
        self.motion_client = actionlib.SimpleActionClient('move_state', MotionAction)
        
    def execute(self, goal):
        fixed = False
        retries = 5
        error_message = PresentGoal()
        retry_navigation = MotionGoal()
        if goal.text == "navigation":
            error_message.text = f"Error in navigation. Robot obstructed, will retry. If you're in the way, please move. {retries} remaining." 
            retry_navigation.goal_x = goal.goal_x
            retry_navigation.goal_y = goal.goal_y
            while fixed == False:
                if retries > 0:
                    self.voice_client.send_goal(error_message)
                    self.voice_client.wait_for_result()
                    retries -= 1
                    self.motion_client.send_goal(retry_navigation)
                    if ac.get_state() == actionlib.SimpleGoalState.SUCCEEDED:
                        fixed = True
                        break
                else:
                    error_message.text = "No more retries left. Please find a museum personnel to manually troubleshoot the robot."
                    self.voice_client.send_goal(error_message)
                    self.voice_client.wait_for_result()

            if fixed == True:
                self.server.set_succeeded() 

        elif goal.text == "interactive":  
            pass

rospy.init_node("error_state_server")
server = ErrorState()
rospy.spin()