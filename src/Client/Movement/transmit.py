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
GSPK = 2.5 # Gyro sensor proportional constant

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
        data = self.socket.recv(1024).decode('utf-8')
        return json.loads(data)

    def receive_heading(self):
        message = self.socket.recv(1024).decode('utf-8')
        data = json.loads(message)
        return float(data["current_heading"])

    def send_confirmation(self, message):
        self.confirmation_socket.sendall(message.encode('utf-8'))
        print("Confirmation sent: {}".format(message))

def get_heading(communicator):
    communicator.send_confirmation("update_heading")
    new_heading = communicator.receive_heading()
    print("Heading received: {}".format(new_heading))
    return new_heading


def normalize_angle(angle):
    return (angle + 180) % 360 - 180

def turn_by_angle(turn_angle, gear_ratio, wheel_diameter, track_width, speed=100):
    turn_circumference = math.pi * track_width
    turn_distance = (turn_circumference * abs(turn_angle)) / 360
    wheel_circumference = math.pi * wheel_diameter
    rotations = turn_distance / wheel_circumference
    degrees = rotations * 360 / gear_ratio

    if turn_angle > 0:
        # Turn clockwise
        left_motor.run_angle(speed, degrees, then=Stop.HOLD, wait=False)
        right_motor.run_angle(speed, -degrees, then=Stop.HOLD, wait=True)
    else:
        # Turn counter-clockwise
        left_motor.run_angle(speed, -degrees, then=Stop.HOLD, wait=False)
        right_motor.run_angle(speed, degrees, then=Stop.HOLD, wait=True)
    
    print("Turned {} degrees 'clockwise' if {} > 0 else 'counter-clockwise'".format(turn_angle, turn_angle))

        

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

teeth_driving_gear = 24
teeth_driven_gear = 16
gear_ratio = teeth_driven_gear / teeth_driving_gear

robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
feed = Motor(Port.C)
color = ColorSensor(Port.S1)
gyro = GyroSensor(Port.S2)
touch = TouchSensor(Port.S3)

#communicator = RobotCommunicator(HOST, PORT, CONFIRMATION_PORT)
#communicator.connect_to_server()
#communicator.connect_to_confirmation()

turn_by_angle(90, gear_ratio, WHEEL_DIAMETER, AXLE_TRACK)

'''
while True:
    data = communicator.receive_data()
    current_position = data['current_position']
    current_heading = data['current_heading']
    target_heading = data['target_heading']
    distance = data['distance'] # the distance is in px
    waypoints = data['waypoints_count']
    print("Received data: {}".format(data))

    turn_by_angle(target_heading, gear_ratio, WHEEL_DIAMETER, AXLE_TRACK)
    drive_distance(robot, distance)

    if waypoints > 1:
        communicator.send_confirmation("reached_waypoint")
        continue
    else:
        ev3.beep()
        break
        '''

