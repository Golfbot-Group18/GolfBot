#!/usr/bin/env pybricks-micropython
import socket
import json
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Direction , Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.hubs import EV3Brick
import time





HOST = '192.168.45.135'  # Server IP address
PORT = 12345  # Server port for receiving vectors
CONFIRMATION_PORT = 12346  # Server port for sending confirmation
AXLE_TRACK = 180  # Distance between the wheels
WHEEL_DIAMETER = 55.5  # Diameter of the wheels
GSPK = 2.5 # The speed to which the robot will correct its path

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
        # Adding a timeout to the blocking of the socket
        #self.socket.timeout(0.5)
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

def drive_distance(distance: float, communicator: RobotCommunicator, speed=100):
    gyro.reset_angle(0)
    drive_base.reset()
    communicator.socket.setblocking(False)
    minDistance = 3000
    if distance > 0:
        # This is the same for each of them to though speed is oppesed 
        while distance>3:

            if colorSensor.reflection() >= 1:
                feed.run_time(speed=4000,time=5*1000, then= Stop.COAST, wait= False)
            

            # if the distance is the smallest distance it 
            # has yet received that means it haven't overshot
            if distance <= minDistance:
                minDistance = distance

                print("Driving")
                print("GyroAngle: ",gyro.angle())
                correction = (0 - gyro.angle()) * GSPK
                drive_base.drive(speed, correction)
                wait(10)
                try:
                    data = communicator.receive_data()
                    distance = data.get('distance')
                except: 
                    print("No new data received")
                
            # If the new distance is longer than the 
            # previous one then it needs to back up
            else:
                print("starting to drive")
                correction = (0 - gyro.angle()) * GSPK
                drive_base.drive(-speed, correction)
                wait(10)
                try:
                    data = communicator.receive_data()
                    distance = data.get('distance')
                except: 
                    print("No new data received")


        

        #drive_base.stop()
        #leftMotor.brake()
        #rightMotor.brake()
    '''    
    else:
        while distance > 2:
        
    '''
            
    
    drive_base.stop()
    leftMotor.brake()
    rightMotor.brake()

    communicator.socket.setblocking(True)
        

'''
def drive_distance(distance, communicator, speed=200):
    drive_base.reset()
    while drive_base.distance() < distance and touchSensor.pressed() is False:
        heading_update = communicator.receive_data().get('heading')
        if heading_update is not None:
            print("New heading: ", heading_update)
            correction = heading_update * 2  # Proportional gain, adjust as needed
            drive_base.drive(speed, -correction)
            time.sleep(0.3)
        else:
            print("No new heading: ", heading_update)
        drive_base.drive(speed=speed, turn_rate=0)
        #time.sleep(0.3)
        #wait(10)
    drive_base.stop()
'''

def turn_on_the_spot(target_heading, communicator, speed=100):
    while touchSensor.pressed() is False:
        heading_update = communicator.receive_data().get('heading')

        if heading_update is None:
            break
        print("New heading: ", heading_update)

        current_heading = heading_update  # Assume heading update is the current heading
        turn_angle = target_heading - current_heading

        if abs(turn_angle) < 5:  # If the turn angle is less than 5 degrees, stop turning
            break
        if turn_angle > 0:
            if abs(turn_angle)>10:
                drive_base.drive(0, speed)
                time.sleep(0.5)
            else: 
                drive_base.drive(0, speed)
                time.sleep(0.1)
        else:
            if abs(turn_angle)>10: 
                drive_base.drive(0, -speed)
                time.sleep(0.5)
            else: 
                drive_base.drive(0, -speed)
                time.sleep(0.5)
        #wait(10)
        #time.sleep(0.3)
        drive_base.stop()
    
    drive_base.stop()

def execute_command(vector, communicator):
    distance_cm, new_heading = vector
    distance_mm = distance_cm * 10
    turn_on_the_spot(new_heading, communicator)
    drive_distance(distance_mm, communicator)
    communicator.send_confirmation("reached")



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
gyro = GyroSensor(Port.S2)
colorSensor = ColorSensor(port=Port.S1)
touchSensor = TouchSensor(Port.S3)

wheel_diameter = 55.5
axle_track = 180

drive_base = DriveBase(leftMotor, rightMotor, wheel_diameter, axle_track)

# Initialize communicator and connect to server
communicator = RobotCommunicator(HOST, PORT, CONFIRMATION_PORT)
communicator.connect_to_server()
communicator.connect_to_confirmation()

# This is the infinite loop where it will take contant updates from the server
while True: 
    # Receive the data and execute the commands 

    # Wait to receive vector data from the server
    data = communicator.receive_data()


    print("Data received:", data)
    #current_heading = data.get('current_heading')
    #target_heading = data.get('target_heading')
    #distance = data.get('distance')
    #vector_count = data.get('vector_count')
    distance = data.get('distance')
    
    drive_distance(distance=distance, communicator=communicator)
    #execute_command((distance, target_heading), communicator)


communicator.close()    

