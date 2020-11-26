# Import Flask along with SQLAlchemy
from flask import Flask, request
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.exc import SQLAlchemyError

# Import OS and read .env files
import os
from os.path import join, dirname
from dotenv import load_dotenv
# app.config.from_object(os.environ['APP_SETTINGS'])
dotenv_path = join(dirname(__file__), '.env')
load_dotenv(dotenv_path)

# Import models and forms to for interacting with the database
from Models import db, Museum, Floor
from validationModels import MuseumModel, FloorModel

app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY')

# Set up Database
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
SQLALCHEMY_DATABASE_URI = os.environ.get('DATABASE_URL')
app.config['SQLALCHEMY_DATABASE_URI'] = SQLALCHEMY_DATABASE_URI
db = SQLAlchemy(app)


@app.route('/')
def hello_world():
    return 'Hello, World!'

# Expects json with values {name: str, floor_count: int}
# Returns
@app.route('/museum/new', methods=['POST'])
def create_museum():
    if request.method == 'POST':
        # Validate Data
        try:
            data = request.get_json()
            model = MuseumModel(
                name=data['name'],
                floor_count=data['floor_count']
            )
        except ValueError as e:
            print(e)
            return e

        print(model.name, model.floor_count)

        museum = Museum(
            name=model.name,
            floor_count=model.floor_count
        )

        # Save to database
        try:
            db.session.add(museum)
            db.session.commit()
            return 'We did it!'
        except SQLAlchemyError as e:
            print(type(e), e)
            return e
    return 'Hello, World!'

# Expects json with values {museum_name: str, level: str }
# Returns
@app.route('/floor/new', methods=['POST'])
def create_floor():
    if request.method == 'POST':
        # Validate Data
        try:
            data = request.get_json()
            model = FloorModel(
                museum_name=data['museum_name'],
                level=data['level'],
            )
        except ValueError as e:
            print(e)
            return e

        floor = Floor(
            museum_name=model.museum_name,
            level=model.level,
        )

        # Save to database
        try:
            db.session.add(floor)
            db.session.commit()
            return 'We did it!'
        except SQLAlchemyError as e:
            print(type(e), e)
            return e
    return 'Hello, World!'

if __name__ == '__main__':
    app.run()