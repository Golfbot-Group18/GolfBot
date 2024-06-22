#!/usr/bin/env pybricks-micropython
import socket
import json
from pybricks.ev3devices import Motor, Gyrosensor
from pybricks.parameters import Port, Direction , Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.hubs import EV3Brick

HOST = '192.168.45.108'  # Server IP address
PORT = 12345  # Server port for receiving vectors
CONFIRMATION_PORT = 12346  # Server port for sending confirmation
AXLE_TRACK = 180  # Distance between the wheels
WHEEL_DIAMETER = 55.5  # Diameter of the wheels

class RobotCommunicator:
    def __init__(self, host, port, confirmation_port):
        self.host = host
        self.port = port
        self.confirmation_port = confirmation_port
        self.socket = None
        self.confirmation_socket = None

    def connect_to_server(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print("Connected to server at {}:{}".format(self.host, self.port))

    def connect_to_confirmation(self):
        self.confirmation_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.confirmation_socket.connect((self.host, self.confirmation_port))
        print("Connected to server for confirmation at {}:{}".format(self.host, self.confirmation_port))

    def receive_data(self):
        data = self.socket.recv(4096)
        data_str = data.decode('utf-8')
        data_json = json.loads(data_str)
        print("Data received:", data_json)
        return data_json

    def send_confirmation(self, message):
        self.confirmation_socket.sendall(message.encode('utf-8'))
        print("Confirmation sent: {}".format(message))

    def close(self):
        if self.socket:
            self.socket.close()
        if self.confirmation_socket:
            self.confirmation_socket.close()
        print("Connection closed")


# Creating the Ev3 brick
ev3 = EV3Brick()

# Initialization beep
ev3.speaker.set_volume(volume=100)
ev3.speaker.set_speech_options(language='en', voice='f1')
ev3.speaker.beep()

# Initialize motors
rightMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE, [24, 16])
leftMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [24, 16])
feed = Motor(Port.C)
gyro = Gyrosensor(Port.S1)

wheel_diameter = 55.5
axle_track = 180

drive_base = DriveBase(leftMotor, rightMotor, wheel_diameter, axle_track)


