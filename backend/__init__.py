# Import Flask along with SQLAlchemy
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.exc import SQLAlchemyError
from flask_cors import CORS

# Import OS and read .env files
import os
from os.path import join, dirname
from dotenv import load_dotenv
# app.config.from_object(os.environ['APP_SETTINGS'])
dotenv_path = join(dirname(__file__), '.env')
load_dotenv(dotenv_path)

# Import all the necessary models
from .routes.auth import auth
from .routes.exhibit import exhibit
from .routes.floor import floor
from .routes.museum import museum
from .routes.piece import piece
from .routes.user import user

def create_app():
    app = Flask(__name__)
    app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY')

    # Set up Database
    app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
    SQLALCHEMY_DATABASE_URI = os.environ.get('DATABASE_URL')
    app.config['SQLALCHEMY_DATABASE_URI'] = SQLALCHEMY_DATABASE_URI
    db = SQLAlchemy(app)

    # Configure Databases for Blueprints
    app.config["museum.db"] = db
    app.config["auth.db"] = db
    app.config["exhibit.db"] = db
    app.config["floor.db"] = db
    app.config["museum.db"] = db
    app.config["piece.db"] = db
    app.config["user.db"] = db

    # Register Blueprints
    app.register_blueprint(auth)
    app.register_blueprint(exhibit)
    app.register_blueprint(floor)
    app.register_blueprint(museum)
    app.register_blueprint(piece)
    app.register_blueprint(user)

    # Enable CORS
    # cors = CORS(app)
    CORS(app, resources={r"/*": {"origins": "*"}})
    app.config['CORS_HEADERS'] = 'Content-Type'
    
    return app