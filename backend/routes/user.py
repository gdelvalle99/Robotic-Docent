from flask import current_app, request, Blueprint
from ..Models import User
from ..validation import UserValidate
from .helpers.valid_login import valid_login
from sqlalchemy.exc import SQLAlchemyError

user = Blueprint('user', __name__, url_prefix="/user")

@user.record
def record(state):
    db = state.app.config.get("user.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through floor.db")

@user.before_request
def before_request():
    valid_login(request)

@user.route('/new', methods=['POST'])
def user_create():
    if request.method == 'POST':
        # Validate Data

        db = current_app.config["user.db"]
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