from flask import current_app, request, Blueprint
from ..Models import Museum
from ..validation import MuseumValidate
from .helpers.valid_login import valid_login
from sqlalchemy.exc import SQLAlchemyError

museum = Blueprint('museum', __name__, url_prefix="/museum")

@museum.record
def record(state):
    db = state.app.config.get("museum.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through museuem.db")

@museum.before_request
def before_request():
    valid_login(request)

# Expects json with values {name: str, floor_count: int}
# Returns a success message depending on if new museum could be made
@museum.route('/new', methods=['POST'])
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

        museum1 = Museum(
            name=model.name,
            floor_count=model.floor_count
        )

        db = current_app.config["museum.db"]

        # Save to database
        try:
            db.session.add(museum1)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new museum"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}
    return {"success": False, "msg": "404 - No Route Found"}

@museum.route('', methods=['GET'])
def get_museum():
    museum_name = request.args.get('name', default = "", type = str)
    db = current_app.config["museum.db"]
    try:
        museum = db.session.query(Museum).filter(Museum.name == museum_name).first()
        serialized_museum = museum.serialize()
        return {"success": True, "museum": serialized_museum}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}
    except Exception as e:
       return {"success": False, "msg": str(e)}