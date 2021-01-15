import requests

url = "http://2bbc3f632f2e.ngrok.io/"
exhibits_url = url + 'floor/exhibits'
pieces_url = url + 'exhibit/pieces'

r = requests.get(exhibits_url)
r_json = r.json()
print(r_json)

r2 = requests.get(pieces_url, params={"id":r_json["exhibits"][0]["id"]})
print(r2.text)