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

web_server = 'http://f7ca7b331704.ngrok.io/'
rospy.init_node('tour_activation')
params = {}
tours = requests.get(web_server, params=params)
while not rospy.is_shutdown:
    for tour in tours:
        if datetime.now() == tour.time:
            requests.get("http://localhost:5000/start_tour", params={"tour_id": tour.tour_id})
        
rospy.spin()