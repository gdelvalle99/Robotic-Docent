from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.exc import SQLAlchemyError
from sqlalchemy.inspection import inspect
import datetime

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
    """Model for the stations table"""
    __tablename__ = 'museums'

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String)
    floor_count = db.Column(db.SmallInteger)

    def __init__(self, name, floor_count):
        self.name = name
        self.floor_count = floor_count


class Floor(BaseModel, db.Model):
    """Model for the stations table"""
    __tablename__ = 'floors'

    id = db.Column(db.Integer, primary_key=True)
    museum_id = db.Column(db.Integer, db.ForeignKey('museums.id'))
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
    """Model for the stations table"""
    __tablename__ = 'exhibits'

    id = db.Column(db.Integer, primary_key=True)
    floor_id = db.Column(db.Integer, db.ForeignKey('floors.id'))
    title = db.Column(db.String)
    subtitle = db.Column(db.String)
    description = db.Column(db.String)
    start_date = db.Column(db.Date)
    end_date = db.Column(db.Date)
    theme = db.Column(db.String)
    questions = db.Column(db.ARRAY(db.String))
    answers = db.Column(db.ARRAY(db.String))

    def __init__(self,floor_id,title,subtitle,description,start_date,theme):
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
            self.theme = theme
            self.questions = []
            self.answers = []
        except SQLAlchemyError as e:
            raise(e)

class Piece(BaseModel, db.Model):
    """Model for the stations table"""
    __tablename__ = 'pieces'

    id = db.Column(db.Integer, primary_key=True)
    exhibit_id = db.Column(db.Integer, db.ForeignKey('exhibits.id'))
    title = db.Column(db.String)
    author = db.Column(db.String)
    description = db.Column(db.String)
    origin = db.Column(db.String)
    era = db.Column(db.String)
    start_date = db.Column(db.Date)
    end_date = db.Column(db.Date)
    acquisition_date = db.Column(db.Date)
    dimension = db.Column(db.ARRAY(db.Float))
    coordinates = db.Column(db.ARRAY(db.Integer))
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
    """Model for the stations table"""
    __tablename__ = 'tours'

    id = db.Column(db.Integer, primary_key=True)
    museum_id = db.Column(db.Integer, db.ForeignKey('museums.id'))
    robot_id = db.Column(db.Integer, db.ForeignKey('robots.id'))
    tour_date = db.Column(db.Date)
    start_time = db.Column(db.Time)
    duration = db.Column(db.Interval)
    interaction_count = db.Column(db.Integer)
    question_count = db.Column(db.Integer)


class Robot(BaseModel, db.Model):
    """Model for the stations table"""
    __tablename__ = 'robots'

    id = db.Column(db.Integer, primary_key=True)
    museum_id = db.Column(db.Integer, db.ForeignKey('museums.id'))
    model = db.Column(db.String)
    tour_count = db.Column(db.BigInteger)
    interaction_count = db.Column(db.BigInteger)
    question_count = db.Column(db.BigInteger)
