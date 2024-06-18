#!/usr/bin/env pybricks-micropython
import json
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from Movement.driveControl import Drive
from Movement.gyroControl import GyroController

import time
import socket

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

HOST = '172.20.10.3'
PORT = 65432

def receive_vectors(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((host, port))
        print("Connected to server at {}:{}".format(host, port))

        # Receive data in chunks and concatenate until the connection is closed
        data = b""
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk

        vectors = json.loads(data.decode('utf-8'))
        print("Vectors received:".format(vectors))
        return vectors
    finally:
        s.close()


# Creating the Ev3 brick
ev3 = EV3Brick()

# Initialization beep
ev3.speaker.beep()