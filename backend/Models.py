from flask_sqlalchemy import SQLAlchemy
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
