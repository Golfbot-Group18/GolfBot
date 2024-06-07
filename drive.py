#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time
import socket


class Drive:
    # Method to run the feedmotor, direction
    #in is feeding into the mechanism and out is out of the robot
    def feedMotor(ev3: EV3Brick, motor: Motor, direction:int):
        #Insert
        if(direction == 1):
            motor.run(-40000)
        #Out
        elif(direction == 0):
            motor.run(40000)
        # This sound will be played if someone made an error in defining the direction
        else:
            Drive.error(ev3)

    # Method to run a given motor in a given direction
    def singleMotor(ev3: EV3Brick, motor: Motor, direction:int):
        #Forwards
        if(direction == 1):
            motor.run(-400000)
        #Backwards
        elif(direction == 0):
            motor.run(400000)
        else:
            Drive.error(ev3)
        
    # Method to run both motors at the same time
    def bothMotors(ev3: EV3Brick, rightMotor:Motor, leftMotor:Motor, direction:int):
        #Backwards
        if(direction == 0):
                leftMotor.run(400)
                rightMotor.run(400)
        #Forwards
        elif(direction == 1):
            leftMotor.run(-400)
            rightMotor.run(-400)
        else:
            Drive.error(ev3)


    # Method to stop any motor. 
    def stopMotor(ev3: EV3Brick, motor: Motor):
        try:
            motor.stop()
        except:
            Drive.error(ev3)

    # Method to stop both motors (intended for drive)
    def stopBothMotors(ev3: EV3Brick, leftMotor:Motor, rightMotor:Motor):
        try:
            leftMotor.stop()
            rightMotor.stop()
        except:
            Drive.error(ev3)

    def turn(ev3:EV3Brick, leftMotor:Motor, rightMotor:Motor, turnangle:int):
        
        print()

    def error(ev3: EV3Brick):
        ev3.speaker.play_notes(['F4/4#','F4/4#','D4/4#','D4/4#'])
        


        