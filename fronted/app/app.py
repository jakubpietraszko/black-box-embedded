from flask import Flask, render_template, request, redirect, url_for, session
import requests
import threading
import time
import sys
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import threading
import time
import paho.mqtt.client as mqtt
import threading
import time
import requests
from flask import Flask, render_template
import paho.mqtt.client as mqtt
import logging
import threading
import time
from flask import Flask
from collections import defaultdict
from flask import Flask, jsonify, render_template

app = Flask(__name__)
app.secret_key = 'your_secret_key'
JSON_SERVER_URL = 'http://localhost:3000'

@app.route('/')
def home():
    return render_template('home.html')


BROKER = "localhost"
PORT = 1883
USERNAME = "user"
PASSWORD = "password"
client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)

mac_data = defaultdict(lambda: {"acc": [], "gyro": [], "temp": []})

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def on_connect(client, userdata, flags, rc):
    logging.info(f"Connected to broker with result code {rc}")
    client.subscribe("bb/#")
    logging.info("Subscribed to all topics under 'bb/'")

MAX_SIZE = 50

def on_message(client, userdata, msg):
    global mac_data
    payload = msg.payload.decode('utf-8')
    logging.info(f"Received message: {payload} on topic {msg.topic}")

    try:
        parts = msg.topic.split('/')
        print(parts)

        _, mac, passwd, data_type = parts[0], parts[1], parts[2], parts[3]

        if data_type == "acc":
            acc_values = list(map(float, payload.split(":")[1].strip().split()))
            mac_data[f"{mac}_{passwd}"]["acc"].append(acc_values)
            if len(mac_data[f"{mac}_{passwd}"]["acc"]) > MAX_SIZE:
                mac_data[f"{mac}_{passwd}"]["acc"].pop(0)
        elif data_type == "gyro":
            gyro_values = list(map(float, payload.split(":")[1].strip().split()))
            mac_data[f"{mac}_{passwd}"]["gyro"].append(gyro_values)
            if len(mac_data[f"{mac}_{passwd}"]["gyro"]) > MAX_SIZE:
                mac_data[f"{mac}_{passwd}"]["gyro"].pop(0)
        elif data_type == "temp":
            temp_value = float(payload.split(":")[1].strip())
            mac_data[f"{mac}_{passwd}"]["temp"].append(temp_value)
            if len(mac_data[f"{mac}_{passwd}"]["temp"]) > MAX_SIZE:
                mac_data[f"{mac}_{passwd}"]["temp"].pop(0)
    except Exception as e:
        logging.error(f"Error processing message in function on_message: {str(e)}")

def background_task():
    while True:
        try:
            logging.info("Collecting data from MQTT topics...")

            if mac_data:
                for key, value in mac_data.items():
                    logging.info(f"MAC ID: {key}")
                    for sensor_type, data in value.items():
                        logging.info(f"Sensor type: {sensor_type}, Data: {data}")
            else:
                logging.info("No data available.")
            print("MAC DATA: ", mac_data)
            time.sleep(5)
        except Exception as e:
            logging.error(f"Error in background task: {str(e)}")
            time.sleep(5)



@app.route('/register', methods=['GET', 'POST'])
def register():
    if request.method == 'POST':
        login = request.form['login']
        password = request.form['password']

        response = requests.post(f'{JSON_SERVER_URL}/users', json={'login': login, 'password': password})
        
        if response.status_code == 201:
            return redirect(url_for('login'))
        else:
            return 'Error: Unable to register user.'
    
    return render_template('register.html')

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        login = request.form['login']
        password = request.form['password']

        users = requests.get(f'{JSON_SERVER_URL}/users').json()
        user = next((u for u in users if u['login'] == login and u['password'] == password), None)

        if user:
            session['user_id'] = user['id']
            return redirect(url_for('dashboard'))
        else:
            return 'Invalid credentials. Please try again.'
    
    return render_template('login.html')
@app.route('/mac_details')
def mac_details():
    if 'user_id' not in session:
        return redirect(url_for('login'))

    mac = request.args.get('mac')
    password = request.args.get('password')

    if not mac or not password:
        return 'Error: MAC address or password missing.'

    return render_template('mac_details.html', mac=mac, password=password)


@app.route('/api/mac/<mac_id>')
def get_mac_data(mac_id):
    print(mac_id)

    if mac_id not in mac_data:
        return jsonify({"error": "MAC ID not found"}), 404

    
    data = mac_data[mac_id]
    
    response = {
        "acc": data["acc"],
        "gyro":  data["gyro"],
        "temp":  data["temp"]
    }
    
    return jsonify(response)


@app.route('/api/mac/<mac_id>/config', methods=['POST'])
def conf(mac_id):
    mac, passwd = mac_id.split('_')
    try:
        data = request.get_json()
        action = data.get('action', '')
        
        if action:
            topic = f"bb/{mac}/{passwd}/config"
            client.publish(topic, payload=action, qos=0, retain=False)
            
            print(f"#####################################MAC Mac: {mac}, Passwd: {passwd}, Action: {action}, Topic: {topic}")
            return jsonify({"success": True, "action": action}), 200
        else:
            return jsonify({"error": "No action specified"}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/add_mac', methods=['POST'])
def add_mac():
    if 'user_id' not in session:
        return redirect(url_for('login'))

    user_id = session['user_id']
    mac = request.form['mac']

    response = requests.post(f'{JSON_SERVER_URL}/macs', json={'userId': user_id, 'mac': mac})

    if response.status_code == 201:
        return redirect(url_for('dashboard'))
    else:
        return 'Error: Unable to add MAC.'

@app.route('/logout')
def logout():
    session.pop('user_id', None)
    return redirect(url_for('login'))


@app.route('/dashboard')
def dashboard():
    if 'user_id' not in session:
        return redirect(url_for('login'))

    user_id = session['user_id']
    blackboxes = requests.get(f'{JSON_SERVER_URL}/blackboxes?userId={user_id}').json()
    return render_template('dashboard.html', blackboxes=blackboxes)

@app.route('/add_blackbox', methods=['POST'])
def add_blackbox():
    if 'user_id' not in session:
        return redirect(url_for('login'))

    user_id = session['user_id']
    mac = request.form['mac']
    password = request.form['password']

    response = requests.post(f'{JSON_SERVER_URL}/blackboxes', json={'userId': user_id, 'mac': mac, 'password': password})

    if response.status_code == 201:
        return redirect(url_for('dashboard'))
    else:
        return 'Error: Unable to add Blackbox.'

@app.route('/update_blackbox/<int:box_id>', methods=['POST'])
def update_blackbox(box_id):
    if 'user_id' not in session:
        return redirect(url_for('login'))

    new_password = request.form['new_password']

    response = requests.patch(f'{JSON_SERVER_URL}/blackboxes/{box_id}', json={'password': new_password})

    if response.status_code == 200:
        return redirect(url_for('dashboard'))
    else:
        return 'Error: Unable to update Blackbox.'

@app.route('/delete_blackbox/<int:box_id>', methods=['POST'])
def delete_blackbox(box_id):
    if 'user_id' not in session:
        return redirect(url_for('login'))

    response = requests.delete(f'{JSON_SERVER_URL}/blackboxes/{box_id}')

    if response.status_code == 200:
        return redirect(url_for('dashboard'))
    else:
        return 'Error: Unable to delete Blackbox.'


if __name__ == '__main__':
    threading.Thread(target=background_task, daemon=True).start()

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()
    
    app.run(debug=True)
