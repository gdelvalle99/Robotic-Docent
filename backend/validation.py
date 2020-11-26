from pydantic import BaseModel, ValidationError, validator
from typing import List
import datetime


class MuseumValidate(BaseModel):
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

class FloorValidate(BaseModel):
    museum_name: str
    level: str

    @validator('museum_name')
    def name_length(cls, v):
        if len(v) <= 0 or len(v) > 128:
            raise ValueError('Museum name must be within 1-128 characters')
        return v

    @validator('level')
    def floor_count_valid(cls, v):
        if len(v) <= 0 or len(v)>8:
            raise ValueError('Level must be within 1-8 characters')
        return v

class ExhibitValidate(BaseModel):
    floor_id: int
    title: str
    subtitle: str
    description: str
    start_date: datetime.date
    theme: str

    @validator('floor_id')
    def id_valid(cls, v):
        if v <= 0:
            raise ValueError('Floor ID is not valid')
        return v

    @validator('title')
    def title_valid(cls, v):
        if len(v) <= 0 or len(v)>32:
            raise ValueError('Title must be within 1-32 characters')
        return v

    @validator('subtitle')
    def subtitle_valid(cls, v):
        if len(v) < 0 or len(v)>64:
            raise ValueError('Subtitle must be within 0-32 characters')
        return v

    @validator('description')
    def description_valid(cls, v):
        if len(v) <= 0 or len(v)>256:
            raise ValueError('Description must be within 1-256 characters')
        return v

    @validator('start_date')
    def start_date_valid(cls, v):
        old_date = datetime.date(1800,1,1)
        if v < old_date:
            raise ValueError('Start date must be valid')
        return v

    @validator('theme')
    def floor_count_valid(cls, v):
        if len(v) <= 0 or len(v)>64:
            raise ValueError('Theme must be within 1-64 characters')
        return v

class PieceValidate(BaseModel):
    exhibit_id: int
    title: str
    author: str
    description: str
    origin: str
    era: str
    acquisition_date: datetime.date
    dimension: List[float]
    coordinates: List[int]

    @validator('exhibit_id')
    def valid_id(cls, v):
        if v <= 0:
            raise ValueError('Exhibit ID is not valid')
        return v

    @validator('title')
    def title_valid(cls, v):
        if len(v) <= 0 or len(v)>32:
            raise ValueError('Title must be within 1-32 characters')
        return v
    
    @validator('author')
    def author_length(cls, v):
        if len(v) <= 0 or len(v) > 64:
            raise ValueError('Author must be within 1-64 characters')
        return v

    @validator('description')
    def description_valid(cls, v):
        if len(v) <= 0 or len(v)>128:
            raise ValueError('Description must be within 1-128 characters')
        return v

    @validator('origin')
    def origin_valid(cls, v):
        if len(v) <= 0 or len(v) > 128:
            raise ValueError('Origin must be within 1-128 characters')
        return v

    @validator('era')
    def era_valid(cls, v):
        if len(v) <= 0 or len(v)>64:
            raise ValueError('Era must be within 1-64 characters')
        return v

    @validator('acquisition_date')
    def date_valid(cls, v):
        old_date = datetime.date(1800,1,1)
        if v < old_date:
            raise ValueError('Acquisition date must be valid')
        return v

    @validator('dimension')
    def dimension_length(cls, v):
        if len(v) != 3:
            raise ValueError('Dimensions must have 3 components (length, width, height)')
        return v

    @validator('coordinates')
    def coordinate_length(cls, v):
        if len(v) != 2:
            raise ValueError('Coordinates must lie on a 2D plane (x,y) ')
        return v