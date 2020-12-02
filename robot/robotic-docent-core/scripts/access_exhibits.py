#!/usr/bin/env python


import Tkinter
import rospy
#Base = declarative_base()
"""
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
"""

# Wait for user to start the tour

# Spawn dialog window

# declare the window
from map_navigation import map_navigation

rospy.init_node('tour_exhibits', anonymous=False)

root = Tkinter.Tk() 


query = {0:
            {
                'coordinates': {
                                    'x':9.97,
                                    'y':13.07
                                },
                'script':'This is exhibit 1'
            },
        1:{
                'coordinates': {
                                    'x':23.48,
                                    'y':17.04
                                },
                'script':'This is exhibit 2'
            },
        2:{
                'coordinates': {
                                    'x':42.47,
                                    'y':17.19
                                },
                'script':'This is exhibit 3'
            },
        3:{
                'coordinates': {
                                    'x':48.07,
                                    'y':41.53
                                },
                'script':'This is exhibit 4'
            }
}

# specify size of window. 
root.geometry("250x170") 

# Create text widget and specify size. 
T = Tkinter.Text(root, height = 5, width = 52) 

map_controller = map_navigation(query)
# Create label 
l = Tkinter.Label(root, text = "Museum Docent") 
l.config(font =("Courier", 14)) 



# Create button for next text. 
b1 = Tkinter.Button(root, text = "Next", command=lambda: map_controller.nextGoal(root, T)) 

# Create an Exit button. 
b2 = Tkinter.Button(root, text = "Exit", 
			command = root.destroy) 

b3 = Tkinter.Button(root, text= "Previous", command = lambda: map_controller.previousGoal(root, T))

l.pack() 
T.pack() 
b1.pack() 
b2.pack()
b3.pack() 

# Insert The Fact. 
T.insert(Tkinter.END, "Starting tour..") 

Tkinter.mainloop() 



