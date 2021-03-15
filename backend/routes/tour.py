from flask import current_app, request, Blueprint
from ..Models import Exhibit, Piece, Tour
from ..validation import ExhibitValidate
from .helpers.valid_login import valid_login
from sqlalchemy.exc import SQLAlchemyError

tour = Blueprint('tour', __name__, url_prefix="/tour")

@tour.record
def record(state):
    db = state.app.config.get("exhibit.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through floor.db")
                        
# @exhibit.before_request
# def before_request():
#     valid_login(request)

# Expects json with values [floor_id: str, title: str, subtitle: str, description: str, start_date: date]
# Returns a json object
@tour.route('/info', methods=['GET'])
def get_tour_info():
    tour_id = request.args.get('tour_id', default="", type = str)
    db = current_app.config["exhibit.db"]

    try:
        tours = db.session.query(Tour).filter(Tour.id==tour_id).all()
        serialized_tour = [i.serialize() for i in tours]
        return {"success": True, "tours": serialized_tour}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}

@tour.route('/piece', methods=['GET'])
def get_tour_piece():
    tour_id = request.args.get('tour_id', default = "", type = str)
    piece_count = request.args.get('piece_count', default = 1, type = int)
    db = current_app.config["exhibit.db"]

    try:
        piece_id = db.session.query(Tour).filter(Tour.id == tour_id).select(Tour.pieces)
        piece_id = piece_id[piece_count]
        piece = db.session.query(Piece).filter(Piece.id == piece_id)
        serialized_piece = [i.serialize() for i in piece]
        return {"success": True, "piece": serialized_piece}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}