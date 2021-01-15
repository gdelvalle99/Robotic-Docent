from flask import Flask, request, make_response
import subprocess
import requests

app = Flask(__name__)



@app.route('/start_tour', methods=['POST'])
def start_tour():
    subprocess.call("./startserver.sh")
    return {
        "success": True
    }

app.run()