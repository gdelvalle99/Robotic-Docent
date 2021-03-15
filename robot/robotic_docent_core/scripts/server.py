#!/usr/bin/env python3

import rospy
from flask import Flask, request
import requests
import json
import actionlib
from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal

app = Flask(__name__)

web_server = 'http://59dd7ccff6fa.ngrok.io/'
web_server_local = 'http://127.0.0.1:5001'
rospy.init_node("server")

# Route for all tours of the day
@app.route('/server/tour', methods=['GET'])
def museum_tours():
    floor_id = request.args.get('floor_id', default = floor_id, type = str)
    museum_id = request.args.get('museum_id', default = museum_id, type = str)
    
    params = {"floor_id": floor_id, "museum_id": museum_id}
    try:
        museum_tours = requests.get(web_server + '/museum/tour', params=params)
        return museum_tours
    except:
        return {"success": False, msg: "Couldn't reach server"}

# Route for a piece
@app.route('/server/piece', methods=['GET'])
def tour_piece():
    tour_id = request.args.get('tour_id', default = tour_id, type = str)
    piece_count = request.args.get('piece_count', default = piece_count, type = int)

    params = {"tour_id": tour_id, "piece_count": piece_count}
    try:
        tour_piece = requests.get(web_server + '/museum/tour/piece', params=params)
        return tour_piece
    except:
        return {"success": False, msg: "Couldn't reach server"}

# Route for an exhibit
@app.route('/server/exhibit', methods=['GET'])
def tour_exhibit():
    #print("hi")
    exhibit_id = request.args.get('exhibit_id', default = "" , type = str)
    params = {"exhibit_id": exhibit_id}
    try:
        tour_exhibit = requests.get(web_server + '/museum/piece/exhibit', params=params)
        return tour_exhibit.json()
    except:
        return {"success": False, msg: "Couldn't reach server"}

# Route for tour info
@app.route('/server/tour/info', methods=['GET'])
def tour_info():
    tour_id = request.args.get('tour_id', default="", type = str)
    params= {"tour_id": tour_id}
    try:
        tour_info = requests.get(web_server + '/museum/tour/info', params=params)
        return tour_exhibit.json()
    except:
        return {"success": False, msg: "Couldn't reach server"}

@app.route('/server/qa', methods=['GET'])
def send_questions():
    f = open("/home/memo/catkin_ws/src/Robotic-Docent/robot/Temp/testqa.json")
    data = json.load(f)
    f.close()
    return data

@app.route('/start_tour', methods=['GET'])
def start_tour():
    tour_id = request.args.get('tour_id', default="", type = str)
    try:
        present_tour_id = PresentGoal()
        present_tour_id.description = tour_id
        # tour_info = requests.get(web_server_local + '/server/tour/info', params={"tour_id": tour_id}).json()
        ac = actionlib.SimpleActionClient('tour_functionality', PresentAction)
        ac.send_goal_and_wait(present_tour_id)
        return {"success": True, msg: "ayy lmao"}
    except:
        return {"success": False, msg: "Couldn't reach server"}


@app.route('/', methods=['GET'])
def hello_world():
    var2 = requests.get(web_server+'/')
    return var2


app.run(debug=True, port=5001)
