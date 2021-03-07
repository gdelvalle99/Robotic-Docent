#!/usr/bin/env python3

import rospy
from flask import Flask, request
import requests

app = Flask(__name__)

web_server = 'http://127.0.0.1:5000'
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
        print("hello")
        tour_exhibit = requests.get(web_server + '/museum/piece/exhibit', params=params)
        return tour_exhibit.json()
    except:
        return {"success": False, msg: "Couldn't reach server"}

# Route for tour info
@app.route('/server/tour/info', methods=['GET'])
def tour_info():
    tour_id = request.args.get('tour_id', default=tour_id, type = str)
    params= {"tour_id": tour_id}
    try:
        tour_info = requests.get(web_server + '/museum/tour/info', params=params)
        return tour_info.json()
    except:
        return {"success": False, msg: "Couldn't reach server"}

@app.route('/', methods=['GET'])
def hello_world():
    print("hi")
    var2 = requests.get(web_server+'/')
    return var2


app.run(debug=True, port=5001)
