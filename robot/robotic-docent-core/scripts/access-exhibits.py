#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

from sqlalchemy import Column, Integer, String
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from map_navigation import map_navigation
from sqlalchemy.orm import sessionmaker
from give-description import show_piece
Base = declarative_base()

class Tour(Base):
    db = rospy.get_param('database')
    tour_name = rospy.get_param('tour')
    engine = create_engine(f'postgresql://localhost/{db}')
    __tablename__ = 'tour'
    id = Column(Integer, primary_key =  True)
    name = Column(String)

    address = Column(String)
    email = Column(String)

# This is pseudocode for querying stuff
Session = sessionmaker(bind = engine)
session = Session()
pieces = Tour.query.filter(exhibit=exhibit).all()

for piece in pieces:
    map_navigation(piece.coordinates[0], piece.coordinates[1])
    show_piece(piece)
    
