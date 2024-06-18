import numpy as np

def calculate_scale_factor_from_ball(diameter_real_cm, ball_data):
    radii = ball_data[0, :, 2]
    average_pixel_radius = np.mean(radii)
    print(f"Average pixel radius: {average_pixel_radius}")
    diameter_pixel = average_pixel_radius * 2
    scale_factor = diameter_real_cm / diameter_pixel
    print(f"Scale factor: {scale_factor}")
    return scale_factor


def camera_calculations():
    pixel_width = 1980
    pixel_height = 1080
    sensor_width = 6 
    sensor_height = 4  
    focal_length = 49   

    fov_x = 2 * np.arctan(sensor_width / (2 * focal_length)) * 180 / np.pi
    fov_y = 2 * np.arctan(sensor_height / (2 * focal_length)) * 180 / np.pi

    print(f"Field of View (FOV) - X: {fov_x:.2f} degrees, Y: {fov_y:.2f} degrees")
    return pixel_width, pixel_height, sensor_width, sensor_height, focal_length

def calculate_real_world_size(size_in_pixels, sensor_size, focal_length, distance_to_object, pixel_dimension):
    size_in_mm = (size_in_pixels * sensor_size) / pixel_dimension
    size_in_real_world = (size_in_mm * distance_to_object) / focal_length
    return size_in_real_world
