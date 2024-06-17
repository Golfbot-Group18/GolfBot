import numpy as np
import matplotlib.pyplot as plt
import math

def convert_path_to_real_world(path, scale_factor):
    return [(point[0] * scale_factor, point[1] * scale_factor) for point in path]

def calculate_distance_and_angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    distance = np.sqrt(dx**2 + dy**2)
    angle = np.degrees(np.arctan2(dy, dx))
    return distance, angle

def generate_vectors_from_path(path_cm):
    vectors = []
    for i in range(1, len(path_cm)):
        distance, angle = calculate_distance_and_angle(path_cm[i-1], path_cm[i])
        vectors.append((distance, angle))
    return vectors

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
                # Add the cumulative distance and turn
                filtered_vectors.append((cumulative_distance, current_angle))
                current_angle = angle
                cumulative_distance = distance
            else:
                cumulative_distance += distance

    if cumulative_distance > distance_threshold:
        filtered_vectors.append((cumulative_distance, current_angle))

    return filtered_vectors