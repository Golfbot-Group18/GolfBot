#!/usr/bin/env pybricks-micropython
import json
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from driveControl import Drive
from gyroControl import GyroController
import time
import socket

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

HOST = '192.168.45.108'
PORT = 65432

def receive_vectors(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print("Trying to connect")
        s.connect((host, port))
        print("Connected to server at {}:{}".format(host, port))

        # Receive data in chunks and concatenate until the connection is closed
        data = b""
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk

        data_str = data.decode('utf-8')
        data_json = json.loads(data_str)
        print("Data received:".format(data_json))
        return data_json
    finally:
        s.close()


# getting the data

received_data = receive_vectors(host=HOST, port=PORT)
robot_heading = received_data['robot_heading']
vectors = received_data['vectors']

print('Robot heading: {}'.format(robot_heading))    
print('Vectors: {}'.format(vectors))


# Creating the Ev3 brick
ev3 = EV3Brick()

# Initialization beep
ev3.speaker.set_volume(volume=100)
ev3.speaker.set_speech_options(language='en', voice='f1')
ev3.speaker.beep()



# initilize motor
rightMotor = Motor(Port.A)
leftMotor = Motor(Port.B)
feed = Motor(Port.C)

# Need a start heading
# at the moment it is just 0 degrees real which is equal to 180
gyroControl = GyroController(Port.S1, robot_heading)


drive = Drive(left_motor_port=Port.B, right_motor_port=Port.A, feed_motor_port=Port.C, gyroController=gyroControl)

counter = 0
for distance, angle in vectors: 
    print('Turning the amount of degrees: {}'.format(angle))
    drive.turn_to_angle(target_angle=angle, speed=60)
    print('Driving the distance: {}'.format(distance))
    drive.run(distance=distance)
    counter += 1
    print('Completed {} sets of instructions'.format(counter))

print('Done with all instructions received.')


ev3.speaker.say("My masters are gonna kill me after this, please help")

    