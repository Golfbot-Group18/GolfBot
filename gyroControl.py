from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

class GyroController:
    def __init__(self, port: Port):
        self.gyro = GyroSensor(port)
        self.reset()
    
    def reset(self):
        self.gyro.reset_angle(0)
        print(self.get_angle())
        wait(500)

    def get_angle(self):

        return self.gyro.angle()
    
    def get_turn_direction(self, target_angle):
        current_angle = self.get_angle()
        if target_angle > current_angle:
            return 1  # Turn right
        else:
            return -1  # Turn left
    
    def angle_difference(self, target_angle):
        return target_angle - self.get_angle()
        