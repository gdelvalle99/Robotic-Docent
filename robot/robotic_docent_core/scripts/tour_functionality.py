#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
from datetime import datetime
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import json

from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal

class Tour:
    def __init__(self):
        # initiliaze
        self.init_publisher = rospy.Publisher("init_publisher", String, queue_size=1)
        print("Initializing tour")
        self.server = actionlib.SimpleActionServer("tour_functionality", PresentAction, self.execute, False)
        self.server.start()

    def execute(self, tour_id):
        print("Executing")
        current_piece = 0
        print("Starting states")
        status_publisher = rospy.Publisher('tour_status_func', String, queue_size=1)
        #self.init_publisher.publish("Initializing states")
        start_client = actionlib.SimpleActionClient('start_state', PresentAction)
        move_client = actionlib.SimpleActionClient('move_state', MotionAction)
        present_client = actionlib.SimpleActionClient('present_state', PresentAction)
        interactive_client = actionlib.SimpleActionClient('interactive_state', PresentAction)

        # Start state
        piece_msg = PresentGoal()
        start_msg = PresentGoal()
        start_msg.description = "Hello! The tour will be starting soon."
        self.init_publisher.publish("Starting up tour.")
        print("Sending requests")

        params = {"tour_id": tour_id.description}

        self.init_publisher.publish(str(tour_id.description))

        tour_info = requests.get('http://ff1d2eede89c.ngrok.io/tour/info', params=params)
        tour_info = tour_info.json()
        tour_length = len(tour_info['tours']['pieces'])
        start_client.send_goal_and_wait(start_msg)
        piece_coordinates = MotionGoal()

        # Loop for exhibits
        for step in range(tour_length):
            self.init_publisher.publish("At piece " + str(step))
            piece_params = {"piece_id": tour_info['tours']['pieces'][step]}
            next_piece = requests.get('http://ff1d2eede89c.ngrok.io/piece', params = piece_params).json()

            cache_file = open("/home/memo/catkin_ws/src/Robotic-Docent/robot/Temp/log.txt", "w+")
            cache_file.write(str(next_piece))

            piece_coordinates.goal_x = next_piece['piece']['coordinates'][0]
            piece_coordinates.goal_y = next_piece['piece']['coordinates'][1]
            piece_msg.description = next_piece['piece']['description']

            self.init_publisher.publish("Sending motion instructions")
            move_client.send_goal_and_wait(piece_coordinates)
            self.init_publisher.publish("Sending present instructions")
            present_client.send_goal_and_wait(piece_msg)

            self.cache_questions(next_piece['piece']['questions'], next_piece['piece']['answers'])
            interactive_client.send_goal_and_wait(piece_msg)
        self.server.set_succeeded()

    def cache_questions(self, piece_questions, piece_answers):
        data = {}
        data['QAList'] = []
        for piece_question, piece_answer in zip(piece_questions, piece_answers):
            data['QAList'].append({"question": piece_question, "answer": piece_answer})
        cache_file = open("/home/memo/catkin_ws/src/Robotic-Docent/robot/Temp/testqa.json", "w+")
        json.dump(data, cache_file)
    

rospy.init_node('main_docent')
tour_server = Tour()
main_docent_publisher = rospy.Publisher("main_docent_publisher", String, queue_size=1)
main_docent_publisher.publish("Node is activated")
# try:
#     tour = Tour()
#     tours = tour.initialize()
#     while len(tours) > 0:
#         current_tour = tours[0]
#         if current_tour.time() == datetime.now().time():
#             tour.execute(current_tour.tour_id)
# except:
#     pass

rospy.spin()

# General workflow:
# When robot is booted, query database for tours of the day
# Constantly check if the time is correct
# If time == tour time, then execute script:
# Let the audience know that the tour will start in a set amount of time (state : start mode)
# Query first piece. Robot starts motion. (state : moving mode. cannot be interacted with.)
# If robot can't reach location, have it stand still (state : error. cannot be interacted with. calls attendant)
# When robot gets to first piece, robot starts lecturing (state : lecturing. cannot be interacted with.)
# 