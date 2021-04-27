import sys
import requests

# python3 send_map.py <web_server_ip> <map_file_name> <museum_name> <floor_id>

server_route = sys.argv[1]
map_file_name = sys.argv[2]
museum_name = sys.argv[3]
floor_id = sys.argv[4]

web_museum = requests.get(server_route + 'museum', params= {"name": museum_name})
if web_museum.json().get("success") == False:
    check = requests.post(web_museum + "museum/new", data={"name": museum_name, "floor_count": floor_id})
    if check.json().get("success") == False:
        print("Something went wrong when creating the museum, please check the server connection.")
        return

web_floor = requests.get(server_route + 'floor', params={"museum_name": museum_name})
if web_museum.json().get("success") == False:
    check = requests.post(server_route + 'floor/new', data={"museum_name": museum_name, "level": floor_id}) 
    if check.json().get("success") == False:
        print("Something went wrong when creating the floor, please check the server connection.")
        return

web_floor = requests.get(server_route + 'floor', params={"museum_name": museum_name, "level": floor_id})
data = {"map": open(map_file_name, "rb"), "floor_id": web_floor.json().get("floor_id")}
requests.post(server_route + 'map/set', data=data)