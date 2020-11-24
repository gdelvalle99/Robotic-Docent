from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.exc import SQLAlchemyError
import os

app = Flask(__name__)
# app.config.from_object(os.environ['APP_SETTINGS'])
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
SQLALCHEMY_DATABASE_URI = os.environ['DATABASE_URL']
app.config['SQLALCHEMY_DATABASE_URI'] = SQLALCHEMY_DATABASE_URI
db = SQLAlchemy(app)

from Models import db, Museum

@app.route('/')
def hello_world():
    return 'Hello, World!'

@app.route('/museum/new')
def create_museum():
    try:
        museum = Museum(
            name="Keck",
            floor_count=3
        )
        db.session.add(museum)
        db.session.commit()
        return 'Hello, World!'
    except SQLAlchemyError as e:
        print(type(e))
        return e
    return 'Hello, World!'


# @app.route('/exhibit/new')
# def create_exhibit():
#     return 'Hello, World!'

if __name__ == '__main__':
    app.run()