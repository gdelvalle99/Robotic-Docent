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

            token = user.encode_auth_token(user.id, current_app.config['SECRET_KEY'])
            return {"success": True, "msg": "Login was successful!", "auth_token": token}
        except SQLAlchemyError as e:
            return {"success": False, "msg": "Username and Password Combo Do Not Match"}
        except Exception as e:
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "Something is broken"}

@auth.route('/validate', methods=['GET'])
def validate_token():
    s = current_app.config.get('SECRET_KEY')
    auth = User.decode_auth_token(request.headers['Authentification'], s)
    if not auth['success']:
        return {"success": False, "msg": auth["msg"]}
    return {"success": True, "msg": "User is logged in!"}