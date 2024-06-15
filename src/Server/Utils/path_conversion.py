import numpy as np

def convert_path_to_real_world(path, scale_factor):
    return [(point[0] * scale_factor, point[1] * scale_factor) for point in path]

def calculate_distance_and_angle(p1, p2):
    """Calculate the distance and angle between two points."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    distance = np.sqrt(dx**2 + dy**2)
    angle = np.degrees(np.arctan2(dy, dx))
    return distance, angle

def generate_vectors_from_path(path_cm):
    """Generate vectors (distance and angle) from the path in cm."""
    vectors = []
    for i in range(1, len(path_cm)):
        distance, angle = calculate_distance_and_angle(path_cm[i-1], path_cm[i])
        vectors.append((distance, angle))
    return vectors