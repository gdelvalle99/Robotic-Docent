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

from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal

class Tour:
    def __init__(self):
        # initiliaze
        self.server = actionlib.SimpleActionServer("tour_functionality", PresentAction, self.execute, False)
        self.server.start()
    
    # def initialize(self):
    #     init_publisher = rospy.Publisher('start_up', String, queue_size=1)
    #     params = [None, None]
    #     Make a request for the tour
    #     tours = requests.get('http://localhost:5000/server/tour', params=params)

    #     init_publisher.publish("Success!")
    #     return tours

    def execute(self, tour_id):
        current_piece = 0

        status_publisher = rospy.Publisher('status', String, queue_size=1)
        start_client = actionlib.SimpleActionClient('start_state', PresentAction)
        move_client = actionlib.SimpleActionClient('move_state', MotionAction)
        present_client = actionlib.SimpleActionClient('present_state', PresentAction)
        interactive_client = actionlib.SimpleActionClient('interactive_state', PresentAction)

        # Start state
        piece_msg = PresentGoal()
        start_msg = "Hello! The tour will be starting soon."
        status_publisher.publish("Starting up tour.")
    
        params = {"tour_id": tour_id}
        tour_info = requests.get('http://localhost:5000/tour/info', params=params)
        tour_length = len(tour_info['pieces'])
        start_client.send_goal_and_wait(start_msg)
        piece_coordinates = MotionGoal()

        # Loop for exhibits
        for step in range(tour_length):
            piece_params = {"tour_id": tour_id, "piece_count": step}
            next_piece = requests.get('http://localhost:5000/server/piece', params = piece_params)
            piece_coordinates.goal_x = next_piece['coordinates'][0]
            piece_coordinates.goal_y = next_piece['coordinates'][1]
            piece_msg.description = next_piece['description']
            move_client.send_goal_and_wait(piece_coordinates)
            present_client.send_goal_and_wait(piece_msg)
            self.cache_questions(piece_msg.questions, piece_msg.answers)
            interactive_client.send_goal_and_wait(piece_msg)

    def cache_questions(self, piece_questions, piece_answers):
        data = {}
        data['QAList'] = []
        for piece_question, piece_answer in zip(piece_questions, piece_answers):
            data['QAList'].append({"question": piece_question, "answer": piece_answer})
        cache_file = open("../Temp/qa.json", "w+")
        json.dump(data, cache_file)
    

rospy.init_node('main_docent')
tour_server = Tour()

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