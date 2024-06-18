from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from gyroControl import GyroController
from pybricks.robotics import DriveBase
import math
import time

class Drive:
    # Initializing motors
    def __init__(self, left_motor_port: Port, right_motor_port: Port, feed_motor_port: Port, gyroController: GyroController):
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.feed_motor = Motor(feed_motor_port)
        self.gyroController = gyroController
    
    # Running for a certain time with a certain speed. 
    def run(self, speed, duration: int):
        self.left_motor.run(speed)
        self.right_motor.run(speed)
        # Duration in seconds
        wait(duration*1000)
        self.left_motor.stop()
        self.right_motor.stop()

    
    def run(self, distance: float): 

        # Wheel ratio 
        # Motor has 24 sprocekts and turning wheel has 16
        # When Motor has turned once then the turning wheel has turned 1,5 times
        # Therefore to turn the wheel once multiply the distance with (2/3)


        # Need to verify the measurements for drivebase
        driveBase = DriveBase(left_motor=self.left_motor, right_motor=self.right_motor, wheel_diameter=55.5, axle_track=180)

        #Using drivbase to turn to the given distance
        distanceConverted = distance*(2/3)*10
        driveBase.straight(distance=-distanceConverted)
        driveBase.stop()


    def run_manual(self, distance: float, speed: int):

        # and know I'll do it manually

        # speed is the degree/s 

        # same wheel ratio as last time. 
        # Distance needs to be multiplied with 
        # (2/3) to get the right distance to run

        # I can use run(speed) and then run at a specific speed for a specific time
        # to get the distance desired, but that didn't work before. 
        # anyway if it would work this is how it would look like

        # Setting speed to 360, that equals to 6 degrees a second
        # distance for 1 wheel rotation is equal to the circumfrence 
        # of the wheel. 

        # Wheel circumfrence is 5,55cm *pi = 17,4358392274 cm
        # Each degree is therefore equivilant to 0,0484328867 cm
        # a second at that speed is therefore equivalent to 0,29 cm
        
        # circumfrence of wheel in cm
        circumfrence = (math.pi*5,55)

        # Amount of degrees I want to turn
        desired_deg = (distance/circumfrence*360)*(2/3)

        # Time to turn wheels in seconds 
        amountTime = desired_deg/speed

        # actually running the motors
        self.left_motor.run(speed=-speed)
        self.right_motor.run(speed=-speed)
        time.sleep(amountTime)
        self.left_motor.stop()
        self.right_motor.stop()
    
    def turn_to_angle_old(self, target_angle, speed):
        print("Inside turn_to_angle")
        while abs(self.gyroController.angle_difference(target_angle)) > 1:
            print("Target angle: {}".format(target_angle))
            current_angle = self.gyroController.get_angle()
            angle_diff = self.gyroController.angle_difference(target_angle)
            print("Current angle: {}, Angle difference: {}".format(current_angle, angle_diff))
            direction = self.gyroController.get_turn_direction(target_angle)
            print("Turn direction: {}".format(direction))
            if direction == 1:
                self.left_motor.run(speed)
                self.right_motor.run(-speed)
            else:
                self.left_motor.run(-speed)
                self.right_motor.run(speed)
        self.left_motor.stop()
        self.right_motor.stop()
        self._correct_overshoot(target_angle)

        

    
    def turn_to_angle(self, target_angle, speed):

        # Okay let's rewrite this 

        # Idea is fine, but let's fix the execution 

        # we'll run the loop as long as there is a difference between 
        # the current angle and the desired angle

        # just gonna convert the angle to this. 
        target_angle = 180-target_angle

        angle_diff = self.gyroController.angle_difference(target_angle=target_angle)

        # Gonna do 10 degree increments 
        while(abs(angle_diff)>10):

            # There is a difference between the current and target angle
            # i.e. we need to figure out which way to turn 

            #if the target is to the left 
            if(angle_diff>0):
                # I want to turn 10 degrees

                # time turning is equal to amount of degrees/speed
                self.left_motor.run(speed=speed)
                self.right_motor.run(speed=-speed)

                # I'm not factoring in how long it takes to accelerate 
                time.sleep(10/speed)

                self.left_motor.stop()
                self.right_motor.stop()



            # if the target is to the right
            else: 
                # I want to turn 10 degrees

                # time turning is equal to amount of degrees/speed
                self.left_motor.run(speed=-speed)
                self.right_motor.run(speed=speed)

                # I'm not factoring in how long it takes to accelerate 
                time.sleep(10/speed)

                self.left_motor.stop()
                self.right_motor.stop()

            angle_diff = self.gyroController.angle_difference(target_angle=target_angle)
            #print(f'angledif1: {angle_diff}')
        
        # second round but lower tollerance 
        while(abs(angle_diff)>1):
            # There is a difference between the current and target angle
            # i.e. we need to figure out which way to turn 

            #if the target is to the left 
            if(angle_diff>0):
                # I want to turn 10 degrees

                # time turning is equal to amount of degrees/speed
                self.left_motor.run(speed=speed)
                self.right_motor.run(speed=-speed)

                # I'm not factoring in how long it takes to accelerate 
                time.sleep(1/speed)

                self.left_motor.stop()
                self.right_motor.stop()



            # if the target is to the right
            else: 
                # I want to turn 10 degrees

                # time turning is equal to amount of degrees/speed
                self.left_motor.run(speed=-speed)
                self.right_motor.run(speed=speed)

                # I'm not factoring in how long it takes to accelerate 
                time.sleep(1/speed)

                self.left_motor.stop()
                self.right_motor.stop()
            
            angle_diff = self.gyroController.angle_difference(target_angle=target_angle)
            #print(f'angledif2: {angle_diff}')

        # and then it should be there

        
    
    
    def _correct_overshoot(self, target_angle):
        while abs(self.gyroController.angle_difference(target_angle)) > 0.5:
            if self.gyroController.get_angle() < target_angle:
                self.left_motor.run(50)
                self.right_motor.run(-50)
            else:
                self.left_motor.run(-50)
                self.right_motor.run(50)
            wait(100)
            self.left_motor.stop()
            self.right_motor.stop()