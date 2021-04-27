from flask import current_app, request, Blueprint
from ..Models import Exhibit, Piece, Tour, TourPieces
from ..validation import ExhibitValidate
from .helpers.valid_login import valid_login
from sqlalchemy.exc import SQLAlchemyError

tour = Blueprint('tour', __name__, url_prefix="/tour")

@tour.record
def record(state):
    db = state.app.config.get("tour.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through floor.db")
                        
# @exhibit.before_request
# def before_request():
#     valid_login(request)

@tour.route('/new', methods=["POST"])
def create_tour():
    db = current_app.config["tour.db"]
    try:
        tour = Tour()
        db.session.add(tour)
        db.session.commit()
        return {"success": True, "msg": "Successfully created a new tour"}  
    except Exception as e:
        return {"success": False, "msg": str(e)}

# Expects json with values [floor_id: str, title: str, subtitle: str, description: str, start_date: date]
# Returns a json object
@tour.route('/info', methods=['GET'])
def get_tour_info():
    tour_id = request.args.get('tour_id', default="", type = str)
    db = current_app.config["tour.db"]

    try:
        tour = db.session.query(Tour).filter(Tour.id==tour_id).first()
        serialized_tour = tour.serialize()

        tour_pieces = db.session.query(TourPieces).filter(TourPieces.tour_id==tour_id).all()
        serialized_tour_pieces = [i.serialize() for i in tour_pieces]
        pieces = [obj['piece_id'] for obj in serialized_tour_pieces]
        serialized_tour['pieces'] = pieces

        return {"success": True, "tours": serialized_tour}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}
    except Exception as e:
        print(e, tour_id)
        return {"success": False, "msg": str(e)}


@tour.route('/piece/add', methods=['POST'])
def add_tourpiece():
    db = current_app.config["tour.db"]
    try:
        data = request.get_json()
        tour_id = data['tour_id']
        piece_id = data['piece_id']

        tour = db.session.query(Tour).filter(Tour.id==tour_id).first()
        piece = db.session.query(Piece).filter(Piece.id==piece_id).first()
        if(tour is not None and piece is not None):

            tour_piece = TourPieces(tour_id, piece_id)

            db.session.add(tour_piece)
            db.session.commit()

        return {"success": True, "msg": "Successfully linked piece to a tour"}
    except Exception as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}

@tour.route('/piece', methods=['GET'])
def get_tour_piece():
    tour_id = request.args.get('tour_id', default = "", type = str)
    piece_count = request.args.get('piece_count', default = 1, type = int)
    db = current_app.config["exhibit.db"]

    try:
        piece_id = db.session.query(TourPieces).filter(TourPieces.tour_id == tour_id).all()
        piece_id = piece_id[piece_count]
        piece = db.session.query(Piece).filter(Piece.id == piece_id)
        serialized_piece = [i.serialize() for i in piece]
        return {"success": True, "piece": serialized_piece}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}

@tour.route('/analytics/interaction', methods=['POST'])
def add_interaction_count():
    db = current_app.config["tour.db"]
    try:
        data = request.get_json()
        tour_id = data['tour_id']
        interaction_count = data['interaction_count']

        tour = db.session.query(Tour).filter(Tour.id==tour_id).first()
        if(tour is not None):
            tour.interaction_count = interaction_count
            db.session.commit()

        return {"success": True, "msg": "Successfully linked piece to a tour"}
    except Exception as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}
