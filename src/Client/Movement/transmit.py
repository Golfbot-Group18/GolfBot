#!/usr/bin/env pybricks-micropython
import socket
import json
from pybricks.ev3devices import Motor, GyroSensor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.hubs import EV3Brick
import math
import time
import random

HOST = '192.168.45.108'  # Server IP address
PORT = 12345  # Server port for receiving vectors
CONFIRMATION_PORT = 12346  # Server port for sending confirmation
AXLE_TRACK = 180  # Distance between the wheels
WHEEL_DIAMETER = 55.5  # Diameter of the wheels
GSPK = 2.5

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
    
    def request_current_heading(self):
        self.socket.sendall("heading".encode('utf-8'))
        print("Requested current heading")

    def get_current_heading(self):
        self.request_current_heading()
        data = self.receive_data()
        heading = data.get('heading', 0)
        return heading

    def send_confirmation(self, message):
        self.confirmation_socket.sendall(message.encode('utf-8'))
        print("Confirmation sent: {}".format(message))

    def close(self):
        if self.socket:
            self.socket.close()
        if self.confirmation_socket:
            self.confirmation_socket.close()
        print("Connection closed")

def get_current_heading(communicator):
    current_heading = communicator.get_current_heading()
    print("Current heading:", current_heading)
    return current_heading

def turn_to_angle(RobotCommunicator, start_heading, target_angle, wheel_diameter, track_width, speed=100):
    current_heading = start_heading

    angle_to_turn = target_angle - current_heading
    
    angle_to_turn = (angle_to_turn + 180) % 360 - 180
    
    while abs(angle_to_turn) > 1:
        turn_circumference = math.pi * track_width

        turn_distance = (turn_circumference * angle_to_turn) / 360

        wheel_circumference = math.pi * wheel_diameter

        rotations = turn_distance / wheel_circumference

        degrees = rotations * 360

        left_motor.run_angle(speed, degrees, then=Stop.HOLD, wait=False)
        right_motor.run_angle(speed, -degrees, then=Stop.HOLD, wait=False)
        
        time.sleep(0.1)
        
        current_heading = get_current_heading(RobotCommunicator)
        
        angle_to_turn = target_angle - current_heading
        angle_to_turn = (angle_to_turn + 180) % 360 - 180

def drive_distance(robot, distance, speed=100):
    gyro.reset_angle(0)
    if distance > 0:
        while robot.distance() <= distance:
            correction = (0 - gyro.angle()) * GSPK
            robot.drive(speed, correction)
            wait(10)
        robot.stop()
        left_motor.brake()
        right_motor.brake()
    else:
        while robot.distance() <= distance:
            correction = (0 - gyro.angle()) * GSPK
            robot.drive(-speed, correction)
            wait(10)
        robot.stop()
        left_motor.brake()
        right_motor.brake()

# Creating the Ev3 brick
ev3 = EV3Brick()

# Initialization beep
ev3.speaker.set_volume(volume=100)
ev3.speaker.set_speech_options(language='en', voice='f1')
ev3.speaker.beep()

# Initialize motors
right_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE, [24, 16])
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [24, 16])
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
feed = Motor(Port.C)
color = ColorSensor(Port.S1)
gyro = GyroSensor(Port.S2)
touch = TouchSensor(Port.S3)

RobotCommunicator = RobotCommunicator(HOST, PORT, CONFIRMATION_PORT)
RobotCommunicator.connect_to_server()
RobotCommunicator.connect_to_confirmation()

while True:
    data = RobotCommunicator.receive_data()
    current_heading = data['current_heading']
    target_heading = data['target_heading']
    distance = data['distance']
    waypoints = data['waypoints_count']

    turn_to_angle(RobotCommunicator, current_heading, target_heading, WHEEL_DIAMETER, AXLE_TRACK)

    drive_distance(robot, distance)

    if waypoints > 1:
        RobotCommunicator.send_confirmation("reached")
    else:
        ev3.speaker.beep()
        break

