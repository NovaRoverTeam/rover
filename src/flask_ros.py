#! /usr/bin/env python

import os
import rospy
import threading

from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from std_msgs.msg import UInt32
from std_msgs.msg import Int32
from harry_potter.msg import DriveCmd
from multiprocessing import Process, Value
import time
heart_beat_cnt = 0

def ros_callback(msg):
    print(msg)

project_root = os.path.dirname(__file__)
template_path = os.path.join(project_root, './')
threading.Thread(target=lambda: rospy.init_node('flask_server', disable_signals=True)).start() 
#Used to create a node in a non-interfering way
rospy.Subscriber('/listener', Int32, ros_callback) #Create subscriber
pub = rospy.Publisher('/drive_cmd', DriveCmd, queue_size=10) #Create publisher

app = Flask(__name__,template_folder=template_path) #Create app and specify template 
socketio = SocketIO(app)  #Establish socket-io server

#Listens for driveEvent signal across socket io
@socketio.on('driveEvent')
def drive_cmd(message):
    global heart_beat_cnt
    print(message['RPM'])
    msg = DriveCmd()
    msg.rpm = message['RPM'] #Publishes RPM key of the message dictionary
    msg.steer_pct = message['steer']
    pub.publish(msg)
    heart_beat_cnt = 0

@app.route('/') #Routes the home director to 
def home():
    return render_template('index.html') #Uses the pre-made frontend

def heartbeat():
	while True:
		global heart_beat_cnt
		heart_beat_cnt = heart_beat_cnt + 1
		print(heart_beat_cnt)
		if heart_beat_cnt > 5:
			dead_heartbeat()
		time.sleep(1)
def dead_heartbeat():
	msg = DriveCmd()
	msg.rpm = 0
	msg.steer_pct = 0

	print('dead')
    
if __name__ == '__main__':
	thread1 = threading.Thread( target=heartbeat ).start()
	app.run(host='0.0.0.0', port=5000)
	thread1.join()
