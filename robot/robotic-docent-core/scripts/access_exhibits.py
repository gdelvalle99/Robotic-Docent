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
from robotic_docent_core.msg import Piece
rospy.init_node('main_docent')

def initialize():
    init_publisher = rospy.Publisher('start_up', String, queue_size=1)
    params = [None, None]
    # Make a request for the tour
    tours = requests.get('http://localhost:5000/museum/tour', params=params)

    init_publisher.publish("Success!")

    return tours

def tour(tour_id):
    current_piece = 0

    status_publisher = rospy.Publisher('status', String, queue_size=1)
    client = actionlib.SimpleActionClient('robot_docent', Piece)

    piece_msg = Piece()
    piece_msg.description = "Thanks for joining us today. We will be starting the tour soon."
    status_publisher.publish("Starting up tour.")
    
    tour_info = requests.get('http://localhost:5000/tour/info', params=params)
    client.send_goal_and_wait(piece_msg)

    

if __name__ = "main":
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