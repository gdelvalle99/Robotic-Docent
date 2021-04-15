from flask import current_app, request, Blueprint
from ..Models import Piece
from ..validation import PieceValidate
from .helpers.valid_login import valid_login
from sqlalchemy.exc import SQLAlchemyError

piece = Blueprint('piece', __name__, url_prefix="/piece")

@piece.record
def record(state):
    db = state.app.config.get("piece.db")

    if db is None:
        raise Exception("This blueprint expects you to provide "
                        "database access through floor.db")

@piece.before_request
def before_request():
    valid_login(request)

# Expects json with values [id: str]
@piece.route('/delete', methods=['GET'])
def delete_piece():
    if request.method == 'GET':
        
        id = request.args.get('id', default = "", type = str)

        db = current_app.config["piece.db"]

        try:
            existingPiece = db.session.query(Piece).filter(Piece.id==id).one()
            if (existingPiece is not None):
                db.session.delete(existingPiece)
                db.session.commit()
                return {"success": True, "msg": "Successfully deleted the piece"} 
            else:
                return {"success": False, "msg": "Piece does not exist"}
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "404 No Existing Route"}

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
            return {"success": True, "msg": "Successfully created a new piece", "id": str(piece.id)}  
        except SQLAlchemyError as e:
            print(type(e), e)
            return {"success": False, "msg": str(e)}

    return {"success": False, "msg": "404 No Existing Route"}

# Expects json with values [exhibit_id: int]
# Returns a json object
@piece.route('', methods=['GET'])
def get_piece():
    piece_id = request.args.get('piece_id', default = "", type = str)
    db = current_app.config["piece.db"]
    try:
        piece = db.session.query(Piece).filter(Piece.id == piece_id).first()
        serialized_piece = piece.serialize()
        return {"success": True, "piece": serialized_piece}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}

@piece.route('/question/add', methods=['POST'])
def add_question():
    db = current_app.config["piece.db"]
    try:
        data = request.get_json()
        if(data is None):
            raise ValueError("Data is empty")
        piece_id = data['piece_id']
        question = data['question']
        answer = data['answer']
        if(question == "" or answer == ""):
            raise ValueError("Question and Answer must both be filled out")
        piece = db.session.query(Piece).filter(Piece.id == piece_id).first()
        piece.questions.append(question)
        piece.answers.append(answer)
        setattr(piece, 'questions', piece.questions)
        setattr(piece, 'answers', piece.answers)
        db.session.commit()
        return {"success": True, "msg": "Successfully added question and answer", "data": {"question": question, "answer": answer}}
    except SQLAlchemyError as e:
        print(type(e), e)
        return {"success": False, "msg": str(e)}
    except Exception as e:
        print(e)
        return {"success": False, "msg": str(e)}
