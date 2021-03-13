from flask import current_app, request, Blueprint
from ..Models import Exhibit, Piece
from ..validation import ExhibitValidate
from .helpers.valid_login import valid_login
from sqlalchemy.exc import SQLAlchemyError

exhibit = Blueprint('exhibit', __name__, url_prefix="/exhibit")

@exhibit.record
def record(state):
    db = state.app.config.get("exhibit.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through floor.db")
                        
@exhibit.before_request
def before_request():
    valid_login(request)

# Expects json with values [floor_id: str, title: str, subtitle: str, description: str, start_date: date]
# Returns a json object
@exhibit.route('/new', methods=['POST'])
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

        db = current_app.config["exhibit.db"]

        try:
            db.session.add(exhibit)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new exhibit"}  
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "404 No Existing Route"}

# DUMMY ROUTE FOR NOW, ONLY RETREIVES PIECES FOR EXHIBITS WITH ID 1, SHOULD CHANGE
# Returns a json object with a list of exhibits given a set floor
@exhibit.route('/pieces', methods=['GET'])
def get_exhibit_pieces():
    id = request.args.get('id', default = "", type = str)
    db = current_app.config["exhibit.db"]
    try:
        pieces = db.session.query(Piece).filter(Piece.exhibit_id==id).all()
        serialized_pieces= [i.serialize() for i in pieces]
        return {"success": True, "pieces": serialized_pieces}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}