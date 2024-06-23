#!/usr/bin/env pybricks-micropython
import json
from pybricks.hubs import EV3Brick
import math
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from gyroControl import GyroController

import time
import socket
'''
def execute_commands(target_heading, distance, priority_ball, vector_count):
    new_heading = target_heading
    distance_mm = distance * 10

    #turn_on_the_spot(new_heading, communicator)
    drive_distance(distance_mm, priority_ball, vector_count)

    #communicator.send_confirmation("reached")

def drive_distance(distance, priority_ball,vector_count, speed=200):
    drive_base.reset()
    while drive_base.distance() < distance:
        #heading_update = communicator.receive_data().get('current_heading')
        if colorSensor.reflection()>0 and priority_ball is False:
            feed.run(4000)
        #if heading_update is not None:
            #correction = heading_update * 2
        drive_base.drive(speed, 0)
        wait(10)
    drive_base.stop()
    if(vector_count==1):
        feed.run_time(speed=4000, time=5*1000, then=Stop.COAST, wait=False)

'''

def turn_on_the_spot(target_heading, current_heading, speed=100):
    while True:

        #heading_update = communicator.receive_data().get('heading')
        #if heading_update is None:
        #    break

        #current_heading = heading_update  # Assume heading update is the current heading
        turn_angle = target_heading - current_heading
        

        if abs(turn_angle) < 5:  # If the turn angle is less than 5 degrees, stop turning
            break

        if turn_angle > 0:
            drive_base.drive(0, speed)
        else:
            drive_base.drive(0, -speed)

        wait(10)
    
    drive_base.stop()

def drive_distance(distance, speed=200):
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


def execute_command(vector, currentHeading):
    distance_cm, new_heading = vector
    distance_mm = distance_cm * 10

    turn_on_the_spot(new_heading, current_heading)
    drive_distance(distance_mm)

    #communicator.send_confirmation("reached")



ev3 = EV3Brick()

ev3.speaker.set_volume(volume=100)
ev3.speaker.set_speech_options(language='en', voice='f1')
ev3.speaker.beep()

rightMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE, [24, 16])
leftMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [24, 16])
feed = Motor(Port.C)

#gyro = GyroSensor(Port.S1)

wheel_diameter = 55.5
axle_track = 180

drive_base = DriveBase(leftMotor, rightMotor, wheel_diameter, axle_track)

rightMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE, [24, 16])
leftMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [24, 16])
feed = Motor(Port.C)
colorSensor = ColorSensor(Port.S1)

wheel_diameter = 55.5
axle_track = 180

drive_base = DriveBase(leftMotor, rightMotor, wheel_diameter, axle_track)


# This is to pick up balls and continue 5 seconds after having reached destination 
#feed.run(speed= 4000)
#feed.run_time(speed=4000, time=5*1000, then=Stop.COAST, wait=False)
#ev3.speaker.beep()





def normalize_angle(angle):
    """Normalize angle to be within the -180 to 180 degree range."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


#print("Data received:", data)
current_heading = normalize_angle(151.4403795216802)
target_heading = normalize_angle(-150.5118841395172)
distance = 37.13531112670898
vector_count = 2

execute_command((distance, target_heading), current_heading)





# This is to figure out the threshold value of the colorSensor for it to start the feed motor.
#while True: 
    #reflectionValue = colorSensor.reflection()
    #print(reflectionValue)

#White ball
#1
#Orange ball
#1
#Obstacle 
#1, but will go above 20


'''
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


'''