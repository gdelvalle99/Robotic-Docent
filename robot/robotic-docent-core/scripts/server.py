import rospy
from flask import Flask
from __init__ import create_app

app = create_app()


# Route for all tours of the day
@app.route('/museum/tour', methods=['GET'])
def museum_tours(floor_id):
    pass

# Route for a piece
@app.route('/tour/piece', methods=['GET'])
def tour_piece(tour_id, piece_count):
    pass

# Route for an exhibit
@app.route('/tour/exhibit', methods=['GET'])
def tour_exhibit(piece_id):
    pass

# Route for tour info
@app.route('/tour/info', methods=['GET'])
def tour_info(tour_id):
    pass

rospy.init_node("server")

if __name__ == "main":
    app.run()
