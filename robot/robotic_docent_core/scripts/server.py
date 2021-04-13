#!/usr/bin/env python3

import rospy
from flask import Flask, request
import requests
import json
import actionlib
from robotic_docent_core.msg import Piece, MotionAction, MotionGoal, PresentAction, PresentGoal, QAAction, QAGoal
from std_msgs.msg import String

app = Flask(__name__)

web_server = 'http://59dd7ccff6fa.ngrok.io/'
web_server_local = 'http://127.0.0.1:5001'
rospy.init_node("server")
tour_ac = actionlib.SimpleActionClient("tour_functionality", PresentAction)
queue_ac = actionlib.SimpleActionClient("queue_server", QAAction)

tour_ac.wait_for_server()
queue_ac.wait_for_server()

@app.route('/send_answer', methods=['POST'])
def send_answer():
    qa_answer = request.get_json()
    f = open("/home/memo/catkin_ws/src/Robotic-Docent/robot/Temp/log.txt", "a")
    f.write(str(qa_answer))
    try:
        robot_answer = QAGoal()
        robot_answer.question = qa_answer['Q']
        robot_answer.answer = qa_answer['A']
        queue_ac.send_goal(robot_answer)
        queue_ac.wait_for_result()
        return {"success": True, "msg": "Success!"}
    except:
        return {"success": False, "msg": "Couldn't reach server"}

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
        # publisher.publish("Sending to tour_func action server..")
        present_tour_id.text = tour_id
        publisher.publish("Sending to tour_func action server after waiting.")
        tour_ac.send_goal(present_tour_id)
        publisher.publish("Sent to tour_func action server after waiting.")
        tour_ac.wait_for_result()
        return {"success": True, "msg": "Success!"}
    except:
        return {"success": False, "msg": "Couldn't reach server"}



app.run(debug=True, port=5000)
listener()
