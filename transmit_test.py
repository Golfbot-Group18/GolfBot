#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from driveControl import Drive
from gyroControl import GyroController
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Initialize the gyro controller
gyroController = GyroController(Port.S1)

driveFunc = Drive(Port.A, Port.B, gyroController)

ev3.speaker.beep()


#driveFunc.run(-100, 3)
driveFunc.turn_to_angle(-90, 200)
#driveFunc.run(100, 3)
#driveFunc.turn_to_angle(0, 50)

