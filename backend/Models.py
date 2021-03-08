from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.exc import SQLAlchemyError
from sqlalchemy.inspection import inspect
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime, timedelta
import hashlib
import jwt
import random
import string
import uuid

db = SQLAlchemy()

class BaseModel(db.Model):
    """Base data model for all objects"""
    __abstract__ = True

    def __init__(self, *args):
        super().__init__(*args)

    def __repr__(self):
        """Define a base way to print models"""
        return '%s(%s)' % (self.__class__.__name__, {
            column: value
            for column, value in self._to_dict().items()
        })

    def json(self):
        """Define a way to jsonify models, dealing with datetime objects"""
        return {
            column: value if not isinstance(value, datetime.date) else value.strftime('%Y-%m-%d')
            for column, value in self._to_dict().items()
        }

    def serialize(self):
       return {c: getattr(self, c) for c in inspect(self).attrs.keys()}



class Museum(BaseModel, db.Model):
    __tablename__ = 'museums'

    id = db.Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, unique=True)
    name = db.Column(db.String)
    floor_count = db.Column(db.SmallInteger)

    def __init__(self, name, floor_count):
        self.name = name
        self.floor_count = floor_count


class Floor(BaseModel, db.Model):
    __tablename__ = 'floors'

    id = db.Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, unique=True)
    museum_id = db.Column(UUID(as_uuid=True), db.ForeignKey('museums.id'))
    level = db.Column(db.String)
    map = db.Column(db.LargeBinary)

    def __init__(self, museum_name, level):
        try:
            self.map = None

            # Set the museum ID
            museum = db.session.query(Museum).filter(Museum.name==museum_name).first()
            if(museum is None):
                raise ValueError("The museum does not exist")
            self.museum_id = museum.id

            # Check to see if level exists
            existing_level = db.session.query(Floor).filter(Floor.level==level).first()
            if(existing_level is not None):
                raise ValueError(museum_name+" already has that floor "+level)
            self.level = level

        except SQLAlchemyError as e:
            raise(e)


class Exhibit(BaseModel, db.Model):
    __tablename__ = 'exhibits'

    id = db.Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, unique=True)
    floor_id = db.Column(UUID(as_uuid=True), db.ForeignKey('floors.id'))
    title = db.Column(db.String)
    subtitle = db.Column(db.String)
    description = db.Column(db.String)
    start_date = db.Column(db.Date)
    end_date = db.Column(db.Date)
    questions = db.Column(db.ARRAY(db.String))
    answers = db.Column(db.ARRAY(db.String))

    def __init__(self,floor_id,title,subtitle,description,start_date):
        try:
            # Check to see if floor exists
            existing_level = db.session.query(Floor).filter(Floor.id==floor_id).first()
            if(existing_level is None):
                raise ValueError("That floor does not exist")
            self.floor_id = floor_id

            self.title = title
            self.subtitle = subtitle
            self.description = description
            self.start_date = start_date
            self.end_date = None
            self.questions = []
            self.answers = []
        except SQLAlchemyError as e:
            raise(e)

class Piece(BaseModel, db.Model):
    __tablename__ = 'pieces'

    id = db.Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, unique=True)
    exhibit_id = db.Column(UUID(as_uuid=True), db.ForeignKey('exhibits.id'))
    title = db.Column(db.String)
    author = db.Column(db.String)
    description = db.Column(db.String)
    origin = db.Column(db.String)
    era = db.Column(db.String)
    start_date = db.Column(db.Date)
    end_date = db.Column(db.Date)
    acquisition_date = db.Column(db.Date)
    dimension = db.Column(db.ARRAY(db.Float))
    coordinates = db.Column(db.ARRAY(db.Float))
    notes = db.Column(db.ARRAY(db.String))
    questions = db.Column(db.ARRAY(db.String))
    answers = db.Column(db.ARRAY(db.String))

    def __init__(self,exhibit_id,title,author,description,origin,era,acquisition_date,dimension,coordinates):
        try:
            # Check to see if exhibit exists
            existing_level = db.session.query(Exhibit).filter(Exhibit.id==exhibit_id).first()
            if(existing_level is None):
                raise ValueError("That exhibit does not exist")
            self.exhibit_id = exhibit_id

            self.title = title
            self.author = author
            self.description = description
            self.origin = origin
            self.era = era
            self.start_date = None
            self.end_date = None
            self.acquisition_date = acquisition_date
            self.dimension = dimension
            self.coordinates = coordinates
            self.notes = []
            self.questions = []
            self.answers = []
        except SQLAlchemyError as e:
            raise(e)


class Tour(BaseModel, db.Model):
    __tablename__ = 'tours'

    id = db.Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, unique=True)
    museum_id = db.Column(UUID(as_uuid=True), db.ForeignKey('museums.id'))
    robot_id = db.Column(UUID(as_uuid=True), db.ForeignKey('robots.id'))
    tour_date = db.Column(db.Date)
    start_time = db.Column(db.Time)
    duration = db.Column(db.Interval)
    interaction_count = db.Column(db.Integer)
    question_count = db.Column(db.Integer)


class Robot(BaseModel, db.Model):
    __tablename__ = 'robots'

    id = db.Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, unique=True)
    museum_id = db.Column(UUID(as_uuid=True), db.ForeignKey('museums.id'))
    model = db.Column(db.String)
    tour_count = db.Column(db.BigInteger)
    interaction_count = db.Column(db.BigInteger)
    question_count = db.Column(db.BigInteger)

class User(BaseModel, db.Model):
    __tablename__ = 'users'

    id = db.Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, unique=True)
    museum_id = db.Column(UUID(as_uuid=True), db.ForeignKey('museums.id'))
    username = db.Column(db.String, unique=True)
    password_hash = db.Column(db.String)
    permission_level = db.Column(db.Integer)
    last_login = db.Column(db.Date)
    last_edit = db.Column(db.Date)

    def generate_password(self, password=None):
        if(password is None):
            choices = string.ascii_letters + string.digits
            password = ''.join(random.choice(choices) for i in range(14))
        
        return hashlib.sha256(password.encode()).hexdigest()

    def set_password(self, password):
        self.password_hash = self.generate_password(password)

    def check_password(self, password):
        return self.password_hash == self.generate_password(password)

    def encode_auth_token(self, user_id, secret):
        try:
            payload = {
                'exp': datetime.utcnow() + timedelta(hours=12),
                'iat': datetime.utcnow(),
                'sub': str(user_id)
            }
            return jwt.encode(
                payload,
                secret,
                algorithm='HS256'
            )
        except Exception as e:
            print("There was an error")
            return e

    @staticmethod
    def decode_auth_token(auth_token, secret):

        try:
            payload = jwt.decode(auth_token, secret, algorithms='HS256')
            return {"success": True, "data": payload['sub']}
        except jwt.ExpiredSignatureError:
            return {"success": False, "msg":'Signature expired. Please log in again.'}
        except jwt.InvalidTokenError:
            return {"success": False, "msg":'Invalid token. Please log in again.'}

    def __init__(self, username, password, permission_level, museum_id):
        self.username = username
        self.set_password(password)
        self.permission_level = permission_level
        self.museum_id = museum_id
        self.last_login = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
