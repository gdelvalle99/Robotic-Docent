import sys
import requests

# python3 send_map.py <web_server_ip> <map_file_name> <museum_name> <floor_id>

server_route = sys.argv[1]
map_file_name = sys.argv[2]
museum_name = sys.argv[3]
floor_id = sys.argv[4]

print(server_route)
web_museum = requests.get(server_route + 'museum', params= {"name": museum_name})
#print(web_museum.json)
if web_museum.json()["success"] == False:
    check = requests.post(server_route + "museum/new", json={"name": museum_name, "floor_count": floor_id})
    print(check.json)
    if check.json()["success"] == False:
        print("Something went wrong when creating the museum, please check the server connection.")
        

web_floor = requests.get(server_route + 'floor', params={"museum_name": museum_name})
print(web_floor.json()['success'])
print(floor_id)
if web_floor.json()['success'] == False:
    check = requests.post(server_route + 'floor/new', json={"museum_name": museum_name, "level": floor_id}) 
    print(check)
    if check.json()['success'] == False:
        print("Something went wrong when creating the floor, please check the server connection.")
        

web_floor = requests.get(server_route + 'floor', params={"museum_name": museum_name, "level": floor_id})
print(web_floor.json())
files = {"map": open(map_file_name, "rb")} 
data = {"floor_id": web_floor.json()["floor"]["id"]}
result = requests.post(server_route + 'floor/map/set', data=data, files=files)
print(result.json())