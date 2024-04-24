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

# our own class for simple drive to motors
import drive


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# setting up the class of functions that drives motors
driveFunc = drive.Drive


# Creating the Ev3 brick
ev3 = EV3Brick()


# Initialization beep
ev3.speaker.beep()




# initilize motor
rightMotor = Motor(Port.B)
leftMotor = Motor(Port.A)
feed = Motor(Port.C)


# Setting up drivebase and configuring it to low speeds
driveBase = DriveBase(leftMotor, rightMotor,75 ,185)
driveBase.settings(30,30,30,30)









feed.run(40000)
'''
# setting up drivebase
driveBase = DriveBase(leftMotor, rightMotor,75 ,185)
driveBase.settings(30,30,30,30)
#driveBase.stop


#feed.run(40000)

#time.sleep(600)


# method to run single motor
#driveFunc.singleMotor(ev3,leftMotor,10,1)
#driveFunc.singleMotor(ev3,rightMotor,10,0)
# method to run both motors
#driveFunc.bothMotors(ev3, rightMotor, leftMotor, 10)



#driveBase.straight(100)
#time.sleep(10)
#driveBase.turn(360)
#time.sleep(10)
#driveBase.straight(100)



# next part is a simple test run where it will drive forward untill it detects
# a ball with the color sensor, run the feed motor and pick up the ball

color_Sensor = ColorSensor(Port.S2)

driveBase.drive(-30,0)

while(color_Sensor.reflection() == 0):
    print(color_Sensor.reflection())
#driveBase.stop()
driveFunc.singleMotor(ev3,feed, 1)
#driveBase.straight(-50)

# second run/ ball
#driveBase.drive(-30,0)

#while(color_Sensor.reflection() == 0):
#    print(color_Sensor.reflection())
#driveBase.stop()
#driveFunc.singleMotor(ev3,feed, 1)
#driveBase.straight(-50)



ev3.speaker.beep()








'''



'''
color_Sensor = ColorSensor(Port.S2)


while(1):
    color = color_Sensor.color()

    print(color_Sensor.reflection())

'''

#ultra_Sensor = UltrasonicSensor(Port.S1)


#while(1):
#    print(ultra_Sensor.distance())