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
    def feedMotor(ev3: EV3Brick, motor: Motor, run_time, direction):
        if(direction == "forward"):
            motor.run(40000)
        elif(direction == "backwards"):
            motor.run(-40000)
        else:
            ev3.speaker.play_notes(['F4/4#','F4/4#','D4/4#','D4/4#'])


    def singleMotor(ev3: EV3Brick, motor: Motor, direction):
        if(direction>0):
            motor.run(400000)
        else:
            motor.run(-400000)
        
        
    def bothMotors(ev3: EV3Brick, motorRight:Motor, motorLeft:Motor, run_time):
        motorLeft.run(-400)
        motorRight.run(400)

        time.sleep(run_time)

        motorLeft.stop()
        motorRight.stop()

    def motorStop(ev3: EV3Brick, motor: Motor):
        motor.stop()
        


        