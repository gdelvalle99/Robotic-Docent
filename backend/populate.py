# This file essentially shows how the api works while populating the database
import requests
link = 'http://127.0.0.1:5000'


def create_musuem():
    data = {"name": "Reno Museum of Art","floor_count": 4}
    requests.post(link+'/museum/new', json=data)


def create_user():
    payload = {"name": "Reno Museum of Art"} 
    r = requests.get(link+'/museum', params=payload)
    if(r.status_code == 200):
        data = r.json()['museum']
        id = data['id']
        data = {
        "username": "ShermanTest",
        "password": "GodSaveMe1",
        "permission_level": 2,
        "museum_id": id
        }
        r = requests.post(link+'/user/new', json=data)

def create_floor():    
    data = {
    "museum_name": "Reno Museum of Art",
    "level": "1"
    }
    requests.post(link+'/floor/new', json=data)

def create_exhibit():
    payload = {"museum_name": "Reno Museum of Art","level": "1"}
    r = requests.get(link+'/floor', params=payload)
    if(r.status_code == 200):
        data = r.json()['floor']
        id = data['id']
        data = {
        "floor_id": id,
        "title": "Contemporary Art Galley",
        "subtitle": "A collection of modern works",
        "description": "Locally sourced, these works come from Nevadans",
        "start_date": "2021-03-07"
        }
        requests.post(link+'/exhibit/new', json=data)


def create_pieces():
    # Find Exhibit ID
    id = "95a3052a-854d-4823-a91b-5ba97ebd52e7" #im super lazy
    payload = {"museum_name": "Reno Museum of Art","level": "1"}
    r = requests.get(link+'/floor', params=payload)
    if(r.status_code == 200):
        data1 = {
            "exhibit_id": id,
            "title": "Louise Bourgeois",
            "author": "Maman",
            "description": "Standing at a height of 30 meters and evoking the shape of a spider, Maman is one the iconic works of Louise Bourgeois.",
            "origin": "London",
            "era": "1990's",
            "acquisition_date": "2018-02-12",
            "dimension": [10,5.3,9.3],
            "coordinates": [9,13]
        }

        data2 = {
            "exhibit_id": id,
            "title": "I Still Face You",
            "author": "Njideka Akunyili Crosby",
            "description": "A young dynamic artist, Njideka Akunyili Crosby unites her dual cultural experience in her vibrant works of art. After spending her formative years in Nigeria, she moved to America at the age of 16 and now lives and works in Los Angeles.",
            "origin": "Los Angeles",
            "era": "2010's",
            "acquisition_date": "2020-03-9",
            "dimension": [3.7,3.3,2.3],
            "coordinates": [42,17]
        }

        data3 = {
            "exhibit_id": id,
            "title": "Untitled",
            "author": "Keith Haring",
            "description": "From his beginnings as a graffiti artist in the New York subway, Keith Haring began his career with his immediately recognizable figures and patterns. One of his most commonly represented symbols is the heart.",
            "origin": "New York",
            "era": "1980's",
            "acquisition_date": "2019-08-12",
            "dimension": [1,1.3,2.3],
            "coordinates": [47,41]
        }

        data4 = {
            "exhibit_id": id,
            "title": "The Physical Impossibility of Death in the Mind of Someone Living",
            "author": "Damien Hirst",
            "description": "Conserved in formaldehyde, the work The Physical Impossibility of Death in the Mind of Someone Living by Damien Hirst is still today one of the most controversial pieces of contemporary art.",
            "origin": "London",
            "era": "1990's",
            "acquisition_date": "2021-02-24",
            "dimension": [2.1,4.3,3.9],
            "coordinates": [18,39]
        }

        # requests.post(link+'/piece/new', json=data1)
        # requests.post(link+'/piece/new', json=data2)
        # requests.post(link+'/piece/new', json=data3)
        # requests.post(link+'/piece/new', json=data4)


def create_tour():
    requests.post(link+'/tour/new')

def link_tourpieces(tour_id, piece_id):
    requests.post(link+'/tour/piece/add', json={"tour_id": tour_id, "piece_id": piece_id})


def get_piece():
    r = requests.get(link+"/piece?piece_id=981eeb31-2156-42ca-8d94-d6833cf651ae")
#     {
#     "exhibit_id": "8b246654-7d81-4b52-9da4-5605a743909e",
#     "title": "Piece Test 2",
#     "author": "kyle",
#     "description": "description",
#     "origin": "origin from earth",
#     "era": "the oldies",
#     "acquisition_date": "1999-02-12",
#     "dimension": [1,0.3,2.3],
#     "coordinates": [0.1,0.3]
# }

if __name__ == "__main__":
    # create_musuem()
    # create_user()
    # create_floor()
    # create_exhibit()
    # create_pieces()
    id = "dbfee438-e5dd-4056-ae2d-4b7a876d351f"
    link_tourpieces(id, "344a2188-90b2-4001-bfb2-6f83a7c841f9")
    link_tourpieces(id, "b8212fc2-d70a-4cb5-89e4-517019a4e737")
    link_tourpieces(id, "49f04602-ac74-4baa-b7d3-cd744082fbda")
    link_tourpieces(id, "3940eefa-f3bc-4c72-9da6-518af816e6ca")
    # link_tourpieces()