from flask import abort, current_app
from ...Models import User

def valid_login(request):
    return
    if request.method == 'OPTIONS':
        return
    if not 'Authentification' in request.headers:
        print(request.headers)
        abort(401, description="Resource not found")

    s = current_app.config.get('SECRET_KEY')
    auth = User.decode_auth_token(request.headers['Authentification'], s)
    if not auth['success']:
        print(auth['msg'])
        abort(401, description="Resource not found")