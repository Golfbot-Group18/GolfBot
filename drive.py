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
    def singleMotor(ev3: EV3Brick, motor: Motor, run_time):
        motor.run(40000)

        time.sleep(run_time*10)


    def bothMotors(ev3: EV3Brick, motorRight:Motor, motorLeft:Motor, run_time):
        motorLeft.run(4000000)
        motorRight.run(400000)

        time.sleep(run_time)

        motorLeft.hold()
        motorRight.hold()

        