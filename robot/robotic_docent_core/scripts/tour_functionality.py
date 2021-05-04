#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
from datetime import datetime
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Polygon
import json

from robotic_docent_core.msg import Piece, ErrorAction, ErrorGoal, MotionAction, MotionGoal, PresentAction, PresentGoal

class Tour:
    def __init__(self, server_route, start_position):
        # initiliaze
        self.start_position = start_position
        self.server_route = server_route
        self.init_publisher = rospy.Publisher("init_publisher", String, queue_size=1)
        print("Initializing tour")
        self.server = actionlib.SimpleActionServer("tour_functionality", PresentAction, self.execute, False)
        self.server.start()
        self.analytics_client = actionlib.SimpleActionClient("analytics_server", PresentAction)
        self.error_client = actionlib.SimpleActionClient("error_state", ErrorAction)
        self.start_client = actionlib.SimpleActionClient('start_state', PresentAction)
        self.move_client = actionlib.SimpleActionClient('move_state', MotionAction)
        self.present_client = actionlib.SimpleActionClient('present_state', PresentAction)
        self.interactive_client = actionlib.SimpleActionClient('interactive_state', PresentAction)
        self.curr_coord_x = 0
        self.curr_coord_y = 0
        self.move_base_subscriber = rospy.Subscriber('move_base', MoveBaseGoal, self.update_coordinates)

    def execute(self, tour_id):
        status_publisher = rospy.Publisher('tour_status_func', String, queue_size=1)
        self.init_publisher.publish("Initializing states")

        # Start state
        piece_msg = PresentGoal()
        start_msg = PresentGoal()
        piece_coordinates = MotionGoal()
        start_msg.text = "Hello! The tour will be starting soon."
        self.init_publisher.publish("Starting up tour.")
        print("Sending requests")

        params = {"tour_id": tour_id.text}

        self.init_publisher.publish(str(tour_id.text))
        tour_info = requests.get(self.server_route + 'tour/info', params=params)
        tour_info = tour_info.json()
        tour_length = len(tour_info["tours"]['pieces'])            
        # self.analytics_client.send_goal(tour_id)
        # Loop for exhibits
        for step in range(tour_length):
            self.init_publisher.publish("At piece " + str(step))
            piece_params = {"piece_id": tour_info['tours']['pieces'][step]}
            next_piece = requests.get(self.server_route + 'piece', params = piece_params).json()

            piece_coordinates.goal_x = next_piece['piece']['coordinates'][0]
            piece_coordinates.goal_y = -next_piece['piece']['coordinates'][1]
            piece_msg.text = next_piece['piece']['description']

            self.init_publisher.publish("Sending motion instructions")
            self.move_client.send_goal(piece_coordinates)
            self.move_client.wait_for_result()  
            if self.move_client.get_state() == 4:    
                self.init_publisher.publish("In error state")
                # self.init_publisher.publish(str(self.curr_coord_x) + " " + str(self.curr_coord_y))
                error_message = ErrorGoal()
                error_message.current_position_x = self.curr_coord_x
                error_message.current_position_y = self.curr_coord_y
                error_message.text = 'navigation'
                error_message.goal_x = piece_coordinates.goal_x
                error_message.goal_y = -piece_coordinates.goal_y
                self.error_client.send_goal(error_message)
                self.error_client.wait_for_result()


            self.init_publisher.publish("Sending present instructions") 
            self.present_client.send_goal_and_wait(piece_msg)

            self.cache_questions(next_piece['piece']['questions'], next_piece['piece']['answers'])
            self.interactive_client.send_goal_and_wait(piece_msg)
        self.analytics_client.send_goal(tour_id)
        self.move_client.send_goal(self.start_position)
        self.move_client.wait_for_result()  
        if self.move_client.get_state() == 4:    
            self.init_publisher.publish("In error state")
            # self.init_publisher.publish(str(self.curr_coord_x) + " " + str(self.curr_coord_y))
            error_message = ErrorGoal()
            error_message.current_position_x = self.curr_coord_x
            error_message.current_position_y = self.curr_coord_y
            error_message.text = 'navigation'
            error_message.goal_x = self.start_position.goal_x
            error_message.goal_y = self.start_position.goal_y
            self.error_client.send_goal(error_message)
            self.error_client.wait_for_result()
        self.server.set_succeeded()

    def cache_questions(self, piece_questions, piece_answers):
        data = {}
        data['QAList'] = []
        for piece_question, piece_answer in zip(piece_questions, piece_answers):
            data['QAList'].append({"question": piece_question, "answer": piece_answer})
        cache_file = open("/home/memo/catkin_ws/src/Robotic-Docent/robot/robotic_docent_core/temp/testqa.json", "w+")
        json.dump(data, cache_file)
    
    def update_coordinates(self, goal):
        self.curr_coord_x = goal.pose.position.x 
        self.curr_coord_y = goal.pose.position.y

web_server = 'http://738e20ddd67c.ngrok.io/'
start_coordinates = MotionGoal()
start_coordinates.goal_x = 77
start_coordinates.goal_y = -67

rospy.init_node('main_docent')
tour_server = Tour(web_server, start_coordinates)
main_docent_publisher = rospy.Publisher("main_docent_publisher", String, queue_size=1)
main_docent_publisher.publish("Node is activated")
rospy.spin()
