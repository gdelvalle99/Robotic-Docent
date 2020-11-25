# from flask_wtf import FlaskForm
# from wtforms import BooleanField, IntegerField, StringField, validators

# class MuseumForm(FlaskForm):
#     name = StringField('Name', [validators.Length(min=3, max=128)])
#     floor_count = IntegerField('Floor Count', [validators.NumberRange(min=1)])

from pydantic import BaseModel, ValidationError, validator


class MuseumModel(BaseModel):
    name: str
    floor_count: int

    @validator('name')
    def name_length(cls, v):
        if len(v) <= 0 or len(v) > 128:
            raise ValueError('Museum name must be within 1-128 characters')
        return v

    @validator('floor_count')
    def floor_count_valid(cls, v):
        if v <= 0 or v>20:
            raise ValueError('Floor count is not in valid range(1-20)')
        return v