import numpy as np
import matplotlib.pyplot as plt
import math

'''This function converts the path from pixel coordinates to real world coordinates. It's done by multiplying each point in the path with the scale factor.'''
def convert_path_to_real_world(path, scale_factor):
    return [(point[0] * scale_factor, point[1] * scale_factor) for point in path]

'''This function calculates the distance and angle between two points. It's done by calculating the difference in x and y coordinates and then using the Pythagorean theorem to calculate the distance. The angle is calculated using the arctan2 function.'''
def calculate_distance_and_angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    distance = np.sqrt(dx**2 + dy**2)
    angle = np.degrees(np.arctan2(dy, dx))
    return distance, angle

'''This function calculates the heading of the robot based on the base and tip points of the robot.'''
def calculate_robot_heading(base, tip):
    _, heading = calculate_distance_and_angle(base, tip)
    return heading

'''This function generates vectors from the path. The vectors are in the form of (distance, angle) - which means the heading the robot should have and the distance it should travel.'''
def generate_vectors_from_path(path_cm):
    vectors = []
    for i in range(1, len(path_cm)):
        distance, angle = calculate_distance_and_angle(path_cm[i-1], path_cm[i])
        vectors.append((distance, angle))
    return vectors

'''Used to filter the insane amount of vectors created from the path'''
def filter_vectors(vectors, distance_threshold=1.0, angle_threshold=5.0):
    filtered_vectors = []
    cumulative_distance = 0
    current_angle = None

    for distance, angle in vectors:
        if current_angle is None:
            current_angle = angle
            cumulative_distance = distance
        else:
            if abs(angle - current_angle) > angle_threshold:
                filtered_vectors.append((cumulative_distance, current_angle))
                current_angle = angle
                cumulative_distance = distance
            else:
                cumulative_distance += distance

    if cumulative_distance > distance_threshold:
        filtered_vectors.append((cumulative_distance, current_angle))

    return filtered_vectors