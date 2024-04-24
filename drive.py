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
    def feedMotor(ev3: EV3Brick, motor: Motor, run_time, direction:str):
        if(direction.lower == "in"):
            motor.run(40000)
        elif(direction.lower == "out"):
            motor.run(-40000)
        # This sound will be played if someone made an error in defining the direction
        else:
            ev3.speaker.play_notes(['F4/4#','F4/4#','D4/4#','D4/4#'])

    # Method to run a given motor in a given direction
    def singleMotor(ev3: EV3Brick, motor: Motor, direction:str):
        if(direction.lower == "backwards"):
            motor.run(400000)
        elif(direction.lower == "backwards"):
            motor.run(-400000)
        else:
            ev3.speaker.play_notes(['F4/4#','F4/4#','D4/4#','D4/4#'])
        
    # Method to run both motors at the same time
    def bothMotors(ev3: EV3Brick, motorRight:Motor, motorLeft:Motor):
        motorLeft.run(-400)
        motorRight.run(400)

    # Method to stop any motor. 
    def motorStop(ev3: EV3Brick, motor: Motor):
        motor.stop()
        


        