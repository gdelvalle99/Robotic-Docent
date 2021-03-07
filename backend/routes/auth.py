from flask import current_app, request, Blueprint
from ..Models import User
from ..validation import UserValidate
from sqlalchemy.exc import SQLAlchemyError

auth = Blueprint('auth', __name__, url_prefix="/auth")

@auth.record
def record(state):
    db = state.app.config.get("auth.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through floor.db")

@auth.route('/login', methods=['POST'])
def user_login():
    if request.method == 'POST':

        db = current_app.config["auth.db"]
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