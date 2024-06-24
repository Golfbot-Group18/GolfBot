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
import threading

HOST = '192.168.197.108'  # Server IP address
PORT = 12345  # Server port for receiving vectors
REQUEST_PORT = 12346  # Server port for sending confirmation
AXLE_TRACK = 180  # Distance between the wheels
WHEEL_DIAMETER = 55.5  # Diameter of the wheels
GSPK = 3 # Gyro sensor proportional constant

class RobotCommunicator:
    def __init__(self, host, port, request_port):
        self.host = host
        self.port = port
        self.request_port = request_port
        self.socket = None
        self.request_socket = None

    def connect_to_server(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print("Connected to server at {}:{}".format(self.host, self.port))

    def connect_to_request(self):
        self.request_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.request_socket.connect((self.host, self.request_port))
        print("Connected to server for request at {}:{}".format(self.host, self.request_port))

    def receive_data(self):
        data = self.socket.recv(1024).decode('utf-8')
        return json.loads(data)

    def receive_heading(self):
        message = self.socket.recv(1024).decode('utf-8')
        data = json.loads(message)
        return float(data["current_heading"])
    
    def receive_position_and_heading(self):
        message = self.socket.recv(1024).decode('utf-8')
        data = json.loads(message)
        return data
    
    def receive_front_clear(self):
        message = self.socket.recv(1024).decode('utf-8')
        data = json.loads(message)
        return data['front_clear']
    
    def receive_backup_point(self):
        message = self.socket.recv(1024).decode('utf-8')
        data = json.loads(message)
        return data['backup_point']

    def send_request(self, message):
        self.request_socket.sendall(message.encode('utf-8'))
        print("Request sent: {}".format(message))

def get_heading(communicator):
    communicator.send_request("update_heading")
    new_heading = communicator.receive_heading()
    print("Heading received: {}".format(new_heading))
    return new_heading


def normalize_angle(angle):
    """Normalize angle to be within the range [-180, 180) degrees."""
    return (angle + 180) % 360 - 180

def calculate_target_heading(current_position, target_position):
    dx = target_position[0] - current_position[0]
    dy = target_position[1] - current_position[1]
    return math.degrees(math.atan2(dy, dx))

def get_distance(current_position, target_position):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((current_position[0] - target_position[0])**2 + (current_position[1] - target_position[1])**2)

def calculate_distance_and_heading(current_position, target_position):
    dx = target_position[0] - current_position[0]
    dy = target_position[1] - current_position[1]
    distance = math.sqrt(dx**2 + dy**2)
    heading = math.degrees(math.atan2(dy, dx))
    return distance, heading

def turn_by_angle(communicator, initial_heading, turn_angle, gear_ratio, wheel_diameter, track_width, speed=150):
    current_heading = initial_heading
    remaining_angle = turn_angle
    increment_angle = 20

    while abs(remaining_angle) > 3: 
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

        communicator.send_request("update_heading")
        current_heading = round(communicator.receive_heading(), 1)

        actual_turn = normalize_angle(current_heading - initial_heading)

        remaining_angle = normalize_angle(turn_angle - actual_turn)
        remaining_angle = round(remaining_angle, 1)

        print("Turned to: {} degrees, remaining angle: {} degrees".format(current_heading, remaining_angle))
    print("Final heading: {}".format(current_heading))
    return current_heading

def check_color_sensor_stability(sensor, stable_value=1, check_duration=3, check_interval=0.1):
    print("Checking color sensor stability...")
    start_time = time.time()
    while time.time() - start_time < check_duration:
        print(sensor.reflection())
        if sensor.reflection() != stable_value:
            return False
        time.sleep(check_interval)
    return True

def adjust_motor_speed(left_motor_speed, right_motor_speed, heading_error):
    correction_factor = 1
    if heading_error > 0:  # Robot is veering left, slow down left motor or speed up right motor
        print("Heading error > 0 - Veering left")
        left_motor_speed += min(heading_error * correction_factor, left_motor_speed - 10)  # Ensure speed doesn't go below a threshold
        print("Left motor speed + : {}".format(left_motor_speed))
        right_motor_speed -= min(heading_error * correction_factor, 1000 - right_motor_speed)  # Ensure speed doesn't exceed max
        print("Right motor speed - : {}".format(right_motor_speed))
    elif heading_error < 0:  # Robot is veering right, do the opposite
        print("Heading error < 0 - Veering right")
        left_motor_speed -= min(abs(heading_error) * correction_factor, 1000 - left_motor_speed)
        print("Left motor speed - : {}".format(left_motor_speed))
        right_motor_speed += min(abs(heading_error) * correction_factor, right_motor_speed - 10)
        print("Right motor speed + : {}".format(right_motor_speed))
    return left_motor_speed, right_motor_speed

def drive_distance_in_intervals(communicator, initial_position, current_heading, target_position, gear_ratio, wheel_diameter, track_width, interval_distance=250, speed=200):
    print("Starting iterval drive...")
    remaining_distance = get_distance(initial_position, target_position)
    print("Initial position: {}, Target position: {}, Remaining distance: {}".format(initial_position, target_position, remaining_distance))
    current_position = initial_position
    target_heading = round(calculate_target_heading(current_position, target_position),1)
    print("Target heading: {}".format(target_heading))
    
    while remaining_distance > 100:
        distance_to_drive = min(interval_distance, remaining_distance) #interval distance is in mm, remaining distance is in px
        wheel_circumference = math.pi * wheel_diameter
        rotations = distance_to_drive / wheel_circumference
        degrees = rotations * 360

        print("Driving distance: {}, using degrees {}".format(distance_to_drive, degrees)) #this is in mm

        heading_error = normalize_angle(target_heading - current_heading)
        print("Current Heading: {}, Heading error: {}".format(current_heading, heading_error))
        adjusted_left_speed, adjusted_right_speed = adjust_motor_speed(speed, speed, heading_error)
        print("Adjusted left speed: {}, Adjusted right speed: {}".format(adjusted_left_speed, adjusted_right_speed))

        left_motor.run_angle(adjusted_left_speed, degrees, then=Stop.HOLD, wait=False)
        right_motor.run_angle(adjusted_right_speed, degrees, then=Stop.HOLD, wait=True)

        # Request updated heading and position
        communicator.send_request("update_pos_and_heading")
        data = communicator.receive_position_and_heading()
        current_position = data['current_position']
        current_heading = data['current_heading']

        print("New position: {}, New heading: {}".format(current_position, current_heading))
        
        remaining_distance = round(get_distance(current_position, target_position),1)
        target_heading = round(calculate_target_heading(current_position, target_position), 1)
        actual_turn = round(normalize_angle(target_heading - current_heading),)
        
        print("New remaining distance: {}, New target heading: {}, Actual turn: {}".format(remaining_distance, target_heading, actual_turn)) #actual turn means the difference between the target heading and the current heading
        if abs(actual_turn) > 3: 
            turn_by_angle(communicator, current_heading, actual_turn, gear_ratio, wheel_diameter, track_width)

        print("Traveled distance: {}, Remaining distance: {}, Current heading: {}".format(distance_to_drive, remaining_distance, current_heading))  

    print("Final position: {}, Remaining distance: {} px".format(current_position, remaining_distance))

def calculate_heading_diff(current_heading, target_heading):
    heading_diff = target_heading - current_heading
    return normalize_angle(heading_diff)

def is_target_behind(robot_position, robot_heading, target_position):
    target_heading = math.degrees(math.atan2(target_position[1] - robot_position[1], target_position[0] - robot_position[0]))
    heading_diff = calculate_heading_diff(robot_heading, target_heading)
    if abs(heading_diff) > 90:
        return True, heading_diff
    return False, heading_diff

def should_back_up(current_position, target_position, current_heading):
    target_behind, heading_diff = is_target_behind(current_position, current_heading, target_position)
    distance_to_target = get_distance(current_position, target_position)

    if target_behind:
        print(f"Target is behind the robot. Heading difference: {heading_diff}")

        if distance_to_target < 100:
            return False, heading_diff
        
        communicator.send_request("check_front_clear")
        data = communicator.receive_data()
        front_clear = data.get('front_clear', True)

        if not front_clear:
            print("Front is not clear, backing up.")
            communicator.send_request("backup_point")
            backup_point = communicator.receive_backup_point()
            return True, backup_point

    return False, None
    
def backup_to_point(communicator, current_position, current_heading, backup_point, gear_ratio, wheel_diameter, track_width, speed=100):
    distance_to_backup_point, heading_to_backup_point = calculate_distance_and_heading(current_position, backup_point)
    backup_heading = normalize_angle(heading_to_backup_point + 180)  # Reverse the heading for backing up

    # Turn to face the backup direction
    turn_angle = normalize_angle(backup_heading - current_heading)

    while distance_to_backup_point > 50:  # Stop when within 10 units of the backup point
        distance_to_drive = min(100, distance_to_backup_point)  # Adjust interval as needed
        wheel_circumference = math.pi * wheel_diameter
        rotations = distance_to_drive / wheel_circumference
        degrees = rotations * 360 / gear_ratio

        # Move backward
        left_motor.run_angle(-speed, degrees, then=Stop.HOLD, wait=False)
        right_motor.run_angle(-speed, degrees, then=Stop.HOLD, wait=True)

        # Request updated position
        communicator.send_confirmation("update_position_and_heading")
        data = communicator.receive_position_and_heading()
        current_position = data['current_position']
        current_heading = data['current_heading']

        # Recalculate the distance to the backup point
        distance_to_backup_point, heading_to_backup_point = calculate_distance_and_heading(current_position, backup_point)
        backup_heading = normalize_angle(heading_to_backup_point + 180)  # Reverse the heading for backing up

        turn_angle = normalize_angle(backup_heading - current_heading)
        if abs(turn_angle) > 3:  # Only adjust if the turn angle is significant
            turn_by_angle(communicator, current_heading, turn_angle, gear_ratio, wheel_diameter, track_width)

        print("Current position: {}, Remaining distance: {}, Current heading: {}".format(current_position, distance_to_backup_point, current_heading))

    print("Backup complete. Final position: {}".format(current_position))
    return current_position, current_heading

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

feed_motor_running = False

def color_sensor_thread():
    global feed_motor_running
    while True:
            reflection = color.reflection()
            #print("Color sensor reflection value: {}".format(reflection))
            if reflection > 1 and not feed_motor_running:
                print("Color sensor detected reflection >= 1")
                feed_motor_running = True
                feed.run_time(speed=4000, time=5000, then=Stop.COAST, wait=False)
                left_motor.run_time(100, 2000, then=Stop.HOLD, wait=False)
                right_motor.run_time(100, 2000, then=Stop.HOLD, wait=True)
                feed_motor_running = False
            time.sleep(0.1)

communicator = RobotCommunicator(HOST, PORT, REQUEST_PORT)
communicator.connect_to_server()
communicator.connect_to_request()

color_thread = threading.Thread(target=color_sensor_thread)
color_thread.daemon = True
color_thread.start()


while True:
    data = communicator.receive_data()
    current_position = data['current_position']
    current_heading = data['current_heading']
    target_position = data['target_position']
    waypoints = data['waypoints_count']
    last_trip = data['last_trip']
    print("Received data: {}".format(data))

    target_heading = round(calculate_target_heading(current_position, target_position))
    print("Target heading: {}".format(target_heading))
    turn_angle = round(normalize_angle(target_heading - current_heading))
    print("Turn angle: {}".format(turn_angle))

    backup_needed, backup_info = should_back_up(current_position, target_position, current_heading, communicator)
    if backup_needed:
        print("Backing up...")
        backup_point = backup_info


    current_heading = turn_by_angle(communicator, current_heading, turn_angle, gear_ratio, WHEEL_DIAMETER, AXLE_TRACK)
    print("Turned to target heading")
    print("Init Driving to target position")
    drive_distance_in_intervals(communicator, current_position, current_heading, target_position, gear_ratio, WHEEL_DIAMETER, AXLE_TRACK)

    print("Waypoint number: {}".format(waypoints))
    if waypoints > 1:
        communicator.send_request("reached_waypoint")
        continue
    if last_trip:
        feed.run_time(-4000, 10000, then=Stop.HOLD, wait=False)
        break
    else:
        communicator.send_request("reached_ball_position")
        ev3.speaker.beep()
        continue

