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
        angle_diff = (target_angle - current_angle + 180) % 360 - 180
        return -1 if angle_diff > 0 else 1
    
    def angle_difference(self, target_angle):
        current_angle = self.get_angle()
        return (target_angle - current_angle + 180) % 360 - 180
        