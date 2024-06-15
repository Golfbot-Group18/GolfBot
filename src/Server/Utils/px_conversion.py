import numpy as np

def calculate_scale_factor_from_ball(diameter_real_cm, ball_data):
    radii = ball_data[0, :, 2]
    average_pixel_radius = np.mean(radii)
    print(f"Average pixel radius: {average_pixel_radius}")
    diameter_pixel = average_pixel_radius * 2
    scale_factor = diameter_real_cm / diameter_pixel
    print(f"Scale factor: {scale_factor}")
    return scale_factor

