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
import Piece



def initialize():
    init_publisher = rospy.Publisher('start_up', String, queue_size=1)
    params = [None, None]
    # Make a request for the tour
    tours = requests.get('http://localhost:5000/server/tour', params=params)

    init_publisher.publish("Success!")

    return tours

def tour(tour_id):
    current_piece = 0

    status_publisher = rospy.Publisher('status', String, queue_size=1)
    start_client = actionlib.SimpleActionClient('start_state', String())
    move_client = actionlib.SimpleActionClient('move_state', Piece)
    present_client = actionlib.SimpleActionClient('present_state', Piece)
    interactive_client = actionlib.SimpleActionClient('interactive_state', Piece)

    # Start state
    piece_msg = Piece()
    start_msg = "Hello! The tour will be starting soon."
    status_publisher.publish("Starting up tour.")
    
    params = {"tour_id": tour_id}
    tour_info = requests.get('http://localhost:5000/tour/info', params=params)
    tour_length = len(tour_info['pieces'])
    start_client.send_goal_and_wait(start_msg)

    # Loop for exhibits
    for step in range(tour_length):
        move_client.send_goal_and_wait(piece_msg)
        present_client.send_goal_and_wait(piece_msg)
        interactive_client.send_goal_and_wait(piece_msg)
    # move state
    # present state
    # interactive state

    

if __name__ == "main":
    rospy.init_node('main_docent')
    try:
        tours = initialize()
        while len(tours) > 0:
            current_tour = tours[0]
            if current_tour.time() == datetime.now().time():
                tour(current_tour.tour_id)

    except:
        pass

# General workflow:
# When robot is booted, query database for tours of the day
# Constantly check if the time is correct
# If time == tour time, then execute script:
# Let the audience know that the tour will start in a set amount of time (state : start mode)
# Query first piece. Robot starts motion. (state : moving mode. cannot be interacted with.)
# If robot can't reach location, have it stand still (state : error. cannot be interacted with. calls attendant)
# When robot gets to first piece, robot starts lecturing (state : lecturing. cannot be interacted with.)
# 