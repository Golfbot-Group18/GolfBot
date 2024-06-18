from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

class GyroController:
        # Initializes the gyroscope and sets the current angle 
        # to the value it is told it is at currently
    def __init__(self, port: Port, currentAngle: int):
        self.gyro = GyroSensor(port, direction=Direction.CLOCKWISE)

        #180 is top 
        # 90 is right
        #-90 is left
        # 0 is bottom

        # all should be solved by the following direction
        self.reset(180-currentAngle)


        # Method to set the angle of the robot
        # i.e. what angle the robot thinks it is at
    def reset(self, desiredAngle: int):
        self.gyro.reset_angle(desiredAngle)
        print(self.get_angle()) 
        #wait(500)

        # Method to get the current angle of the robot
    def get_angle(self):

        return self.gyro.angle()


        # A method to get the turn direction to correct itself if not 
        # at the correct angle
    def get_turn_direction(self, target_angle):
        current_angle = self.get_angle()
        if target_angle > current_angle:
            return 1  # Turn right
        else:
            return -1  # Turn left
        


        # Method to get the difference between the current
        # angle and the desired angle
    def angle_difference(self, target_angle):
        return self.get_angle() - target_angle
        