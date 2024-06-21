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
PORT = 12345
CONFIRMATION_PORT = 12346

class RobotCommunicator:
    def __init__(self, host, port, confirmation_port):
        self.host = host
        self.port = port
        self.confirmation_port = confirmation_port
        self.socket = None

    def connect_to_server(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print(f"Connected to server at {self.host}:{self.port}")

    def receive_data(self):
        data = self.socket.recv()
        data_str = data.decode('utf-8')
        data_json = json.loads(data_str)
        print("Data received:", data_json)
        return data_json

    def send_confirmation(self, message):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.host, self.confirmation_port))
            s.sendall(message.encode('utf-8'))
            print(f"Confirmation sent: {message}")

    def close(self):
        if self.socket:
            self.socket.close()
        print("Connection closed")

def drive_distance(distance, communicator, speed=200):
    drive_base.reset()
    while drive_base.distance() < distance:
        heading_update = communicator.receive_data().get('current_heading')
        if heading_update is not None:
            correction = heading_update * 2
            drive_base.drive(speed, -correction)
        wait(10)
    drive_base.stop()

def turn_on_the_spot(target_heading, communicator, speed=100):
    while True:
        heading_update = communicator.receive_data().get('heading')
        if heading_update is None:
            break

        current_heading = heading_update
        turn_angle = target_heading - current_heading

        if abs(turn_angle) < 5:
            break

        if turn_angle > 0:
            drive_base.drive(0, speed)
        else:
            drive_base.drive(0, -speed)

        wait(10)
    
    drive_base.stop()

def execute_commands(target_heading, distance, communicator):
    new_heading = target_heading
    distance_mm = distance * 10

    turn_on_the_spot(new_heading, communicator)
    drive_distance(distance_mm, communicator)

    communicator.send_confirmation("reached")

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

wheel_diameter = 55.5
axle_track = 180

drive_base = DriveBase(leftMotor, rightMotor, wheel_diameter, axle_track)

communicator = RobotCommunicator(HOST, PORT, CONFIRMATION_PORT)
communicator.connect_to_server()


try:
    while True:
        data = communicator.receive_data()
        print("Data received:", data)
        current_heading = data.get('current_heading')
        target_heading = data.get('target_heading')
        distance = data.get('distance')
        vector_count = data.get('vector_count')
        
        execute_commands(target_heading, distance, communicator)
        if vector_count == 1:
            ##pick up ball at end of vector
            feed.run_target(100, 90)
finally:
    communicator.close()

    