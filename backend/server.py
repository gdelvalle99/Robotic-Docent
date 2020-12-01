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

# Import models and forms to for interacting with the database
from Models import db, Museum, Floor, Exhibit, Piece
from validation import MuseumValidate, FloorValidate, ExhibitValidate, PieceValidate

# Image Packages
import io
import PIL.Image as Image

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
            model = MuseumValidate(
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
            return {"success": True, "msg": "Successfully created a new floor"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}
    return {"success": False, "msg": "404 - No Route Found"}

# Expects json with values {museum_name: str, level: str }
# Returns
@app.route('/floor/new', methods=['POST'])
def create_floor():
    if request.method == 'POST':
        # Validate Data
        try:
            data = request.get_json()
            model = FloorValidate(
                museum_name=data['museum_name'],
                level=data['level'],
            )
        except ValueError as e:
            print(e)
            return {"success": False, "msg": str(e)}

        floor = Floor(
            museum_name=model.museum_name,
            level=model.level,
        )

        # Save to database
        try:
            db.session.add(floor)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new floor"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}
    return {"success": False, "msg": "404 No Existing Route"}

# Expects formdata with values [floor_id: int, photo: png ]
# Returns a success message if was able to save map
@app.route('/floor/map', methods=['GET','POST'])
def floor_map():
    if request.method == 'POST':
        # Validate Data
        id = int(request.form['floor_id']) or 0
        if('map' not in request.files):
            return "there's no map file"

        try:
            floor = db.session.query(Floor).filter(Floor.id==id).first()
            if(floor is None):
                raise ValueError("Floor ID doesn't match")

            # Save to database
            floor.map = request.files['map'].read()
            db.session.commit()
            return {"success": False, "msg": "Successfully updated the floor map"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg":e}

    # Get Request
    return {"success": False, "msg": "404 No Existing Route"}

# Expects formdata with values [floor_id: int]
# Returns a png image
@app.route('/floor/update', methods=['POST'])
def floor_update():
    if request.method == 'POST':
        # Validate Data
        id = int(request.form['floor_id']) or 0

        try:
            floor = db.session.query(Floor).filter(Floor.id==id).first()
            if(floor is None):
                raise ValueError("Floor ID doesn't match")

            # Create a response and send the image
            response = make_response(floor.map)
            response.headers.set('Content-Type', 'image/png')
            return response
            
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "404 No Existing Route"}

# 
# 
@app.route('/floor/exhibits', methods=['GET'])
def get_exhibits():
    id = request.args.get('id', default = 2, type = int)
    try:
        exhibits = db.session.query(Exhibit).filter(Exhibit.floor_id==id).all()
        serialized_exhibits = [i.serialize() for i in exhibits]
        # return {"success": True, "exhibits": serialized_exhibits[0]}
        return {"success": True, "exhibits": serialized_exhibits}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}

# Expects json with values [floor_id: int]
# Returns a json object
@app.route('/exhibit/new', methods=['POST'])
def create_exhibit():
    if request.method == 'POST':
        # Validate Data
        try:
            data = request.get_json()
            model = ExhibitValidate(
                floor_id=data['floor_id'],
                title=data['title'],
                subtitle=data['subtitle'],
                description=data['description'],
                start_date=data['start_date'],
                theme=data['theme']
            )
        except ValueError as e:
            print(e)
            return {"success": False, "msg": str(e)}

        exhibit = Exhibit(
            floor_id=model.floor_id,
            title=model.title,
            subtitle=model.subtitle,
            description=model.description,
            start_date=model.start_date,
            theme=model.theme             
        )

        try:
            db.session.add(exhibit)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new exhibit"}  
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "404 No Existing Route"}

# Expects json with values [floor_id: int]
# Returns a json object
@app.route('/piece/new', methods=['POST'])
def create_piece():
    if request.method == 'POST':
        # Validate Data
        try:
            data = request.get_json()
            model = PieceValidate(
                exhibit_id=data['exhibit_id'],
                title=data['title'],
                author=data['author'],
                description=data['description'],
                origin=data['origin'],
                era=data['era'],
                acquisition_date=data['acquisition_date'],
                dimension=data['dimension'],
                coordinates=data['coordinates']
            )
        except ValueError as e:
            print(e)
            return {"success": False, "msg": str(e)}

        piece = Piece(
            exhibit_id=model.exhibit_id,
            title=model.title,
            author=model.author,
            description=model.description,
            origin=model.origin,
            era=model.era,
            acquisition_date=model.acquisition_date,
            dimension=model.dimension,
            coordinates=model.coordinates
        )

        # Save piece into database
        try:
            db.session.add(piece)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new piece"}  
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "404 No Existing Route"}

# 
# 
@app.route('/exhibit/pieces', methods=['GET'])
def get_exhibit_pieces():
    id = request.args.get('id', default = 1, type = int)
    try:
        pieces = db.session.query(Piece).filter(Piece.exhibit_id==id).all()
        serialized_pieces= [i.serialize() for i in pieces]
        return {"success": True, "pieces": serialized_pieces}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}

if __name__ == '__main__':
    app.run()