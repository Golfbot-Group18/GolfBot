import numpy as np
import matplotlib.pyplot as plt
import math

'''This function converts the path from pixel coordinates to real world coordinates. It's done by multiplying each point in the path with the scale factor.'''
def convert_path_to_real_world(path, scale_factor):
    return [(point[0] * scale_factor, point[1] * scale_factor) for point in path]

'''This function calculates the distance and angle between two points. It's done by calculating the difference in x and y coordinates and then using the Pythagorean theorem to calculate the distance. The angle is calculated using the arctan2 function.
def calculate_distance_and_angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    distance = np.sqrt(dx**2 + dy**2)
    angle = np.degrees(np.arctan2(dy, dx))
    return distance, angle
'''

def calculate_distance_and_angle(point1, point2):
    dx = point2[0] - point1[0]
    dy = -(point2[1] - point1[1])
    
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle = math.degrees(math.atan2(dy, dx))
    return distance, angle

def calculate_angle(vector):
    return np.arctan2(vector[1], vector[0]) * 180 / np.pi


'''This function calculates the heading of the robot based on the base and tip points of the robot.'''
def calculate_robot_heading(base, tip):
    _, heading = calculate_distance_and_angle(base, tip)
    return heading

'''This function generates vectors from the path. The vectors are in the form of (distance, angle) - which means the heading the robot should have and the distance it should travel.'''
def convert_path_to_vectors(path):
    vectors = []
    for i in range(len(path) - 1):
        y1, x1 = path[i]
        y2, x2 = path[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        vectors.append((dx, dy))
    return vectors

def simplify_vectors(vectors):
    if not vectors:
        return []
    
    simplified_vectors = []
    current_vector = vectors[0]
    count = 1
    
    for i in range(1, len(vectors)):
        if vectors[i] == current_vector:
            count += 1
        else:
            simplified_vectors.append((current_vector[0] * count, current_vector[1] * count))
            current_vector = vectors[i]
            count = 1
    
    # Append the last accumulated vector
    simplified_vectors.append((current_vector[0] * count, current_vector[1] * count))
    
    return simplified_vectors

def further_simplify_vectors(vectors):
    simplified_vectors = []
    current_vector = vectors[0]
    count = 1
    
    for i in range(1, len(vectors)):
        if vectors[i] == current_vector:
            count += 1
        else:
            if count > 1:
                simplified_vectors.append((current_vector[0] * count, current_vector[1] * count))
            else:
                simplified_vectors.append(current_vector)
            current_vector = vectors[i]
            count = 1
    
    if count > 1:
        simplified_vectors.append((current_vector[0] * count, current_vector[1] * count))
    else:
        simplified_vectors.append(current_vector)
    
    return simplified_vectors

def aggressively_simplify_vectors(vectors):
    if not vectors:
        return []
    
    simplified_vectors = []
    current_vector = vectors[0]
    current_dx = current_vector[0]
    current_dy = current_vector[1]
    
    for i in range(1, len(vectors)):
        if vectors[i][0] == current_vector[0] and vectors[i][1] == current_vector[1]:
            current_dx += vectors[i][0]
            current_dy += vectors[i][1]
        else:
            simplified_vectors.append((current_dx, current_dy))
            current_vector = vectors[i]
            current_dx = current_vector[0]
            current_dy = current_vector[1]
    
    # Append the last accumulated vector
    simplified_vectors.append((current_dx, current_dy))
    
    return simplified_vectors

def combine_small_steps(vectors, tolerance=1):
    if not vectors:
        return []

    simplified_vectors = []
    current_dx, current_dy = vectors[0]
    accumulated_dx, accumulated_dy = current_dx, current_dy
    
    for i in range(1, len(vectors)):
        dx, dy = vectors[i]
        
        # Check if the direction is the same within the tolerance
        if abs(dx - current_dx) <= tolerance and abs(dy - current_dy) <= tolerance:
            accumulated_dx += dx
            accumulated_dy += dy
        else:
            simplified_vectors.append((accumulated_dx, accumulated_dy))
            current_dx, current_dy = dx, dy
            accumulated_dx, accumulated_dy = dx, dy

    # Append the last accumulated vector
    simplified_vectors.append((accumulated_dx, accumulated_dy))
    
    return simplified_vectors

def vectors_to_distance_and_heading(vectors, initial_heading=0):
    distance_and_heading = []
    current_heading = initial_heading
    
    for dx, dy in vectors:
        distance = math.sqrt(dx**2 + dy**2)
        heading = math.degrees(math.atan2(dy, dx))
        
        # Adjust the heading relative to the robot's current heading
        relative_heading = (heading - current_heading) % 360
        
        # Update the robot's current heading
        current_heading = heading
        
        distance_and_heading.append((distance, relative_heading))
    
    return distance_and_heading

