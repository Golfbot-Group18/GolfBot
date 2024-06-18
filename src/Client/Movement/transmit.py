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

HOST = '172.20.10.3'
PORT = 65432

def receive_vectors(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print("Trying to connect")
        s.connect((host, port))
        print("Connected to server at {}:{}".format(host, port))

        data = b""
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk

        vectors = json.loads(data.decode('utf-8'))
        print("Vectors received:".format(vectors))
        return vectors
    finally:
        s.close()


# getting the data

received_vectors = receive_vectors(host=HOST, port=PORT)


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
gyroControl = GyroController(Port.S1, 180)


drive = Drive(left_motor_port=Port.B, right_motor_port=Port.A, feed_motor_port=Port.C, gyroController=gyroControl)

counter = 0
for distance, angle in received_vectors: 
    print(f'Turning the amount of degrees: {angle}')
    drive.turn_to_angle(target_angle=angle, speed=60)
    print(f'Driving the distance: {distance}')
    drive.run(distance=distance)
    counter += 1
    print(f'Completed {counter} sets of instructions')

print(f'Done with all instructions received.')


ev3.speaker.say("My masters are gonna kill me after this, please help")

    