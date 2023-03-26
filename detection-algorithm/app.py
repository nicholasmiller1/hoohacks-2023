import os
from flask import Flask, request
from flask_cors import CORS
from detection import run_detection

app = Flask(__name__)
CORS(app)

@app.route("/")
def index():
    return "GUIDEROOM Processing WebServer"

@app.route("/processing", methods=['POST'])
def processing():
    uploaded_file = request.files['file_model']
    print("Received file:", uploaded_file.filename)

    filepath = f"./var/www/uploads/{uploaded_file.filename}"
    uploaded_file.save(filepath)
    result = run_detection(filepath)

    os.remove(filepath)
    return str(result)