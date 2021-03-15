from flask import current_app, request, make_response, Blueprint
from ..Models import Floor, Exhibit
from ..validation import FloorValidate
from .helpers.valid_login import valid_login
from sqlalchemy.exc import SQLAlchemyError

floor = Blueprint('floor', __name__, url_prefix="/floor")

@floor.record
def record(state):
    db = state.app.config.get("floor.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through floor.db")

@floor.before_request
def before_request():
    valid_login(request)

# Expects json with values {museum_name: str, level: str }
# Returns a success message depending on if a new level could be made
@floor.route('/new', methods=['POST'])
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

        db = current_app.config["floor.db"]

        # Save to database
        try:
            db.session.add(floor)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new floor"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}
    return {"success": False, "msg": "404 No Existing Route"}

# Expects formdata with values [floor_id: str, map: png ]
# Returns a success message if was able to save map
@floor.route('/map/set', methods=['GET','POST'])
def floor_map():
    if request.method == 'POST':
        # Validate Data
        id = str(request.form['floor_id']) or ""
        if('map' not in request.files):
            return "there's no map file"

        db = current_app.config["floor.db"]

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

# Expects formdata with values [floor_id: str]
# Returns a png image of the map
@floor.route('/map/get', methods=['POST'])
def floor_update():
    if request.method == 'POST':
        # Validate Data
        id = str(request.form['floor_id']) or ""

        db = current_app.config["floor.db"]

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
@floor.route('/exhibits', methods=['GET'])
def get_exhibits():
    id = request.args.get('id', default = "", type = str)

    db = current_app.config["floor.db"]

    try:
        exhibits = db.session.query(Exhibit).filter(Exhibit.floor_id==id).all()
        serialized_exhibits = [i.serialize() for i in exhibits]
        return {"success": True, "exhibits": serialized_exhibits}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}