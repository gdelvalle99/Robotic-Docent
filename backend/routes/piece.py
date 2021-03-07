from flask import current_app, request, Blueprint
from ..Models import Piece
from ..validation import PieceValidate
from sqlalchemy.exc import SQLAlchemyError

piece = Blueprint('piece', __name__, url_prefix="/piece")

@piece.record
def record(state):
    db = state.app.config.get("piece.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through floor.db")

# Expects json with values [exhibit_id: int]
# Returns a json object
@piece.route('/new', methods=['POST'])
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

        db = current_app.config["piece.db"]

        # Save piece into database
        try:
            db.session.add(piece)
            db.session.commit()
            return {"success": True, "msg": "Successfully created a new piece"}  
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "404 No Existing Route"}