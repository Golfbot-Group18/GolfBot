import numpy as np
import matplotlib.pyplot as plt
import math

'''This function converts the path from pixel coordinates to real world coordinates. It's done by multiplying each point in the path with the scale factor.'''
def convert_path_to_real_world(path, scale_factor):
    return [(point[0] * scale_factor, point[1] * scale_factor) for point in path]


def calculate_distance_and_angle(point1, point2):
    dx = point2[0] - point1[0]
    dy = -(point2[1] - point1[1])
    
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle = math.degrees(math.atan2(dy, dx))
    return distance, angle


'''This function calculates the heading of the robot based on the base and tip points of the robot.'''
def calculate_robot_heading(base, tip):
    _, heading = calculate_distance_and_angle(base, tip)
    return heading






