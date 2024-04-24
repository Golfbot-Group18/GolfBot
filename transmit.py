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

import drive


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

driveFunc = drive.Drive

#listOfDevices = bluetooth



# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

# initilize motor
rightMotor = Motor(Port.B)
leftMotor = Motor(Port.A)
feed = Motor(Port.C)

#feed.run(40000)

#time.sleep(600)
'''
color_Sensor = ColorSensor(Port.S2)


while(1):
    color = color_Sensor.color()

    print(color_Sensor.reflection())

'''

#ultra_Sensor = UltrasonicSensor(Port.S1)


#while(1):
#    print(ultra_Sensor.distance())


#driveFunc.singleMotor(ev3,feed,4)
driveFunc.bothMotors(ev3, rightMotor, leftMotor, 4)


ev3.speaker.beep()

