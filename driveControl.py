from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from gyroControl import GyroController

class Drive:
    def __init__(self, left_motor_port: Port, right_motor_port: Port, gyroController: GyroController):
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.gyroController = gyroController
    
    def run(self, speed, duration):
        self.left_motor.run(speed)
        self.right_motor.run(speed)
        wait(duration*1000)
        self.left_motor.stop()
        self.right_motor.stop()
    
    def turn_to_angle(self, target_angle, speed):
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