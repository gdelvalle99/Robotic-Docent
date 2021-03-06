# Import Flask along with SQLAlchemy
from flask import Flask, request, make_response
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.exc import SQLAlchemyError
from __init__ import create_app

# Import models and forms to for interacting with the database
from Models import Museum, Floor, Exhibit, Piece, User
from validation import MuseumValidate, FloorValidate, ExhibitValidate, PieceValidate, UserValidate

app = create_app()
db = SQLAlchemy(app)


@app.route('/')
def hello_world():
    return 'Hello, World!'

# Expects json with values {name: str, floor_count: int}
# Returns a success message depending on if new museum could be made
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

        museum = Museum(
            name=model.name,
            floor_count=model.floor_count
        )

        # Save to database
        try:
            db.session.add(museum)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new museum"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}
    return {"success": False, "msg": "404 - No Route Found"}

# Expects json with values {museum_name: str, level: str }
# Returns a success message depending on if a new level could be made
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

# Expects formdata with values [floor_id: int, map: png ]
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
            return {"success": True, "msg": "Successfully updated the floor map"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg":e}

    # Get Request
    return {"success": False, "msg": "404 No Existing Route"}

# Expects formdata with values [floor_id: int]
# Returns a png image of the map
@app.route('/floor/retrieve/map', methods=['POST'])
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

# DUMMY ROUTE FOR NOW, ONLY RETREIVES EXHIBITS FOR FLOORS WITH ID 2, SHOULD CHANGE
# Returns a json object with a list of exhibits given a set floor
@app.route('/floor/exhibits', methods=['GET'])
def get_exhibits():
    id = request.args.get('id', default = 2, type = int)
    try:
        exhibits = db.session.query(Exhibit).filter(Exhibit.floor_id==id).all()
        serialized_exhibits = [i.serialize() for i in exhibits]
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

# DUMMY ROUTE FOR NOW, ONLY RETREIVES PIECES FOR EXHIBITS WITH ID 1, SHOULD CHANGE
# Returns a json object with a list of exhibits given a set floor
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

@app.route('/user/login', methods=['POST'])
def user_login():
    if request.method == 'POST':
        try:
            data = request.get_json()
            username = data['username']
            password = data['password']
            user = db.session.query(User).filter(User.username==username).one()
            if(user is None or not user.check_password(password)):
                raise ValueError("Username and Password Combo Do Not Match")
            return {"success": True, "msg": "Login was successful!"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}
        except ValueError as e:
            return {"success": False, "msg": str(e)}

    return

@app.route('/user/create', methods=['POST'])
def user_create():
    if request.method == 'POST':
        # Validate Data
        try:
            data = request.get_json()
            model = UserValidate(
                username=data['username'],
                password=data['password'],
                permission_level=data['permission_level'],
                museum_id=data['museum_id'],
            )
        except ValueError as e:
            print(e)
            return {"success": False, "msg": str(e)}

        user = User(
            username=model.username,
            password=model.password,
            permission_level=model.permission_level,
            museum_id=model.museum_id,
        )

        # Save piece into database
        try:
            db.session.add(user)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new user"}  
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "404 No Existing Route"}


if __name__ == '__main__':
    app.run()