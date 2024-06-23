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
    """Normalize angle to be within the range [-180, 180) degrees."""
    return (angle + 180) % 360 - 180

def calculate_target_heading(current_position, target_position):
    dx = target_position[1] - current_position[1]
    dy = target_position[0] - current_position[0]
    return math.degrees(math.atan2(dy, dx))

def turn_by_angle(communicator, initial_heading, turn_angle, gear_ratio, wheel_diameter, track_width, speed=150):
    remaining_angle = turn_angle
    increment_angle = 20

    while abs(remaining_angle) > 1: 
        turn_increment = min(abs(remaining_angle), increment_angle) 

        turn_circumference = math.pi * track_width
        turn_distance = (turn_circumference * turn_increment) / 360
        wheel_circumference = math.pi * wheel_diameter
        rotations = turn_distance / wheel_circumference
        degrees = rotations * 360 / gear_ratio

        if remaining_angle > 0:
            # Turn clockwise
            left_motor.run_angle(speed, degrees, then=Stop.HOLD, wait=False)
            right_motor.run_angle(speed, -degrees, then=Stop.HOLD, wait=True)
        else:
            # Turn counterclockwise
            left_motor.run_angle(speed, -degrees, then=Stop.HOLD, wait=False)
            right_motor.run_angle(speed, degrees, then=Stop.HOLD, wait=True)

        communicator.send_confirmation("update_heading")
        current_heading = communicator.receive_heading()

        actual_turn = normalize_angle(current_heading - initial_heading)

        remaining_angle = normalize_angle(turn_angle - actual_turn)

        print("Turned to: {} degrees, remaining angle: {} degrees".format(current_heading, remaining_angle))

    print("Final heading: {}".format(current_heading))

def drive_distance(communicator: RobotCommunicator, target_position, speed=100):
    gyro.reset_angle(0)
    robot.reset()
    communicator.send_confirmation("init_drive")
    print("Waiting for first data in drive")
    data = communicator.receive_data()
    distance = data.get('distance')
    current_heading = data.get('current_heading')
    updated_position = data.get('updated_position')
    print("received first data ")
    communicator.socket.setblocking(False)
    minDistance = 3000
    if distance > 0:
        # This is the same for each of them to though speed is oppesed 
        while distance>3:

            if color.reflection() >= 1:
                feed.run_time(speed=4000,time=5*1000, then= Stop.COAST, wait= False)
            

            # if the distance is the smallest distance it 
            # has yet received that means it haven't overshot
            if distance <= minDistance:
                minDistance = distance

                print("Driving")
                
                #print("GyroAngle: ",gyro.angle())
                print("Current Heading: ",current_heading)
                target_heading = calculate_target_heading(updated_position, target_position)
                print("Target Heading: ",target_heading)
                turn_angle = normalize_angle(target_heading - current_heading)
                print("Turn Angle: ",turn_angle)
                correction = (turn_angle)
                robot.drive(speed, correction)
                wait(10)
                try:
                    data = communicator.receive_data()
                    distance = data.get('distance')
                    current_heading = data.get('current_heading')
                    updated_position = data.get('updated_position')
                except: 
                    print("No new data received")
                
            # If the new distance is longer than the 
            # previous one then it needs to back up
            else:
                print("starting to drive")
                print("Current Heading: ",current_heading)
                target_heading = calculate_target_heading(updated_position, target_position)
                print("Target Heading: ",target_heading)
                turn_angle = normalize_angle(target_heading - current_heading)
                print("Turn Angle: ",turn_angle)
                correction = (turn_angle)
                robot.drive(-speed, correction)
                wait(10)
                try:
                    data = communicator.receive_data()
                    distance = data.get('distance')
                    current_heading = data.get('current_heading')
                    updated_position = data.get('updated_position')
                except: 
                    print("No new data received")


        

        #drive_base.stop()
        #leftMotor.brake()
        #rightMotor.brake()
    '''    
    else:
        while distance > 2:
        
    '''
            
    
    robot.stop()
    communicator.socket.setblocking(True)  

def drive_distance_old(robot, distance, speed=100):
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

communicator = RobotCommunicator(HOST, PORT, CONFIRMATION_PORT)
communicator.connect_to_server()
communicator.connect_to_confirmation()

while True:
    data = communicator.receive_data()
    current_position = data['current_position']
    current_heading = data['current_heading']
    target_position = data['target_position']
    waypoints = data['waypoints_count']
    print("Received data: {}".format(data))

    target_heading = calculate_target_heading(current_position, target_position)
    turn_angle = normalize_angle(target_heading - current_heading)
    print("Turn angle: {}".format(turn_angle))

    turn_by_angle(communicator, current_heading, turn_angle, gear_ratio, WHEEL_DIAMETER, AXLE_TRACK)
    drive_distance(communicator, target_position)

    if waypoints > 1:
        communicator.send_confirmation("reached_waypoint")
        continue
    else:
        ev3.beep()
        break