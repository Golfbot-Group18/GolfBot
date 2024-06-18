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

def drive_distance(distance, speed=200):
    initial_angle = gyro.angle()

    drive_base.reset()
    drive_base.straight(distance)

    while drive_base.distance() < distance:
        current_angle = gyro.angle()
        error = current_angle - initial_angle
        correction = error * 2 
        
        drive_base.drive(speed, -correction)
        wait(10)

    drive_base.stop()

def turn_on_the_spot(degrees, speed=100):
    target_angle = gyro.angle() + degrees

    if degrees > 0:
        drive_base.drive(0, speed)
        while gyro.angle() < target_angle:
            wait(10)
    else:
        drive_base.drive(0, -speed)
        while gyro.angle() > target_angle:
            wait(10)
    
    drive_base.stop()

def execute_commands(vectors):
    current_heading = 0
    for vector in vectors:
        distance_cm, new_heading = vector
        distance_mm = distance_cm * 10

        turn_angle = new_heading - current_heading

        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360

        turn_on_the_spot(turn_angle)

        current_heading = new_heading

        drive_distance(distance_mm)

# Creating the Ev3 brick
ev3 = EV3Brick()

# Initialization beep
ev3.speaker.set_volume(volume=100)
ev3.speaker.set_speech_options(language='en', voice='f1')
ev3.speaker.beep()



# initilize motor
rightMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE, [24, 16])
leftMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [24, 16])
feed = Motor(Port.C)

gyro = GyroSensor(Port.S1)

wheel_diameter = 55.5
axle_track = 180

drive_base = DriveBase(leftMotor, rightMotor, wheel_diameter, axle_track)

gyro.reset_angle(0)

#drive_distance(1000)

data = receive_vectors(HOST, PORT)
print("Data received:", data)
vectors = data['vectors']


execute_commands(vectors)
ev3.speaker.say("I have reached the destination")

    