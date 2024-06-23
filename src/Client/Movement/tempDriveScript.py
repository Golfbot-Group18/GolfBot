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
from driveControl import Drive
from gyroControl import GyroController

import time
import socket
#import driveExecution




# Creating the Ev3 brick
ev3 = EV3Brick()

# Initialization beep
ev3.speaker.beep()

# initilize motor
rightMotor = Motor(Port.A, Direction.CLOCKWISE)
leftMotor = Motor(Port.B, Direction.CLOCKWISE)
feed = Motor(Port.C)

gyroControl = GyroController(Port.S2, 180)

color = ColorSensor(port=Port.S1)

touch = TouchSensor(port=Port.S3)


#drive = Drive(left_motor_port=Port.B, right_motor_port=Port.A, feed_motor_port=Port.C, gyroController=gyroControl)



#print(gyroControl.get_angle())
#drive.turn_to_angle(-90, 60)




#rightMotor.run(-360)
#leftMotor.run(-360)
#feed.run(3600)
#time.sleep(50)
#rightMotor.stop()
#leftMotor.stop()


print(gyroControl.get_angle())

print("ending it all")

#driveExecution.driveForwards(30)

robot = DriveBase(leftMotor, rightMotor, 55.5, 180)

robot.drive(-50,0)
feed.run(4000)

ev3.speaker.set_speech_options(language='en', voice='f1')
ev3.speaker.say("Welcome to hell")

#ev3.speaker.play_notes(['F4/4#','F4/4#','D4/4#','D4/4#'])
