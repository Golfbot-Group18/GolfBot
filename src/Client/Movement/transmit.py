#!/usr/bin/env pybricks-micropython
import socket
import json
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.hubs import EV3Brick

HOST = '192.168.45.108'  # Server IP address
PORT = 12345  # Server port for receiving vectors
CONFIRMATION_PORT = 12346  # Server port for sending confirmation

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

def drive_distance(distance, communicator, speed=200):
    drive_base.reset()
    while drive_base.distance() < distance:
        # Temporarily removed the request for a current heading
        #heading_update = communicator.receive_data().get('heading')
        #if heading_update is not None:
            #correction = heading_update * 2  # Proportional gain, adjust as needed
            #drive_base.drive(speed, -correction)
        drive_base.drive(speed=speed, turn_rate=0)
        wait(10)
    drive_base.stop()

def turn_on_the_spot(target_heading, communicator, speed=100):
    while True:
        heading_update = communicator.receive_data().get('heading')
        if heading_update is None:
            break

        current_heading = heading_update  # Assume heading update is the current heading
        turn_angle = target_heading - current_heading

        if abs(turn_angle) < 5:  # If the turn angle is less than 5 degrees, stop turning
            break

        if turn_angle > 0:
            drive_base.drive(0, speed)
        else:
            drive_base.drive(0, -speed)

        wait(10)
    
    drive_base.stop()

def execute_command(vector, communicator):
    distance_cm, new_heading = vector
    distance_mm = distance_cm * 10

    turn_on_the_spot(new_heading, communicator)
    drive_distance(distance_mm, communicator)

    communicator.send_confirmation("reached")

def pickup_ball():
    ev3.speaker.say("Picking up the ball")
    feed.run_angle(1000, 360)  # Example to run motor to pick up the ball, adjust as needed
    wait(1000)
    feed.stop()

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

wheel_diameter = 55.5
axle_track = 180

drive_base = DriveBase(leftMotor, rightMotor, wheel_diameter, axle_track)

# Initialize communicator and connect to server
communicator = RobotCommunicator(HOST, PORT, CONFIRMATION_PORT)
communicator.connect_to_server()
communicator.connect_to_confirmation()

while True:
    # Wait to receive vector data from the server
    data = communicator.receive_data()
    if not data:
        print("No data received, waiting...")
        continue

    print("Data received:", data)
    current_heading = data.get('current_heading')
    target_heading = data.get('target_heading')
    distance = data.get('distance')
    vector_count = data.get('vector_count')

    execute_command((distance, target_heading), communicator)

    if vector_count == 1:
        # Pick up ball at end of vector
        feed.run_target(100, 90)

communicator.close()
