# Import Flask along with SQLAlchemy
from flask import Flask, request, make_response
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.exc import SQLAlchemyError

# Import OS and read .env files
import os
from os.path import join, dirname
from dotenv import load_dotenv
# app.config.from_object(os.environ['APP_SETTINGS'])
dotenv_path = join(dirname(__file__), '.env')
load_dotenv(dotenv_path)

def create_app():
    app = Flask(__name__)
    app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY')

    # Set up Database
    app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
    SQLALCHEMY_DATABASE_URI = os.environ.get('DATABASE_URL')
    app.config['SQLALCHEMY_DATABASE_URI'] = SQLALCHEMY_DATABASE_URI
    return app