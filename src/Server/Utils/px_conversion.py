import numpy as np
import math

def calculate_scale_factor_from_ball(diameter_real_cm, ball_data):
    radii = ball_data[0, :, 2]
    average_pixel_radius = np.mean(radii)
    print(f"Average pixel radius: {average_pixel_radius}")
    diameter_pixel = average_pixel_radius * 2
    scale_factor = diameter_real_cm / diameter_pixel
    print(f"Scale factor: {scale_factor}")
    return scale_factor



# To account for the height displacement of the robot, all in cm
def realCoordinates (robotHeight_cm: float, cameraHeight_cm: float, robotCoordinates):
    #I'm at the moment assuming that directly beneath the camera is the center of the 
    # grid. This should always be the case if the camera is level. 
    cameraCoordinates = (960,540)

    x_diff = robotCoordinates[0]-cameraCoordinates[0]
    #print(f'x_diff: {x_diff}')
    y_diff = robotCoordinates[1]- cameraCoordinates[1]
    #print(f'y_diff: {y_diff}')

    r = math.sqrt (math.pow(x_diff,2)+math.pow(y_diff,2))
    #print(f'r: {r}')

    if r != 0: 


        delta_f_cm = cameraHeight_cm-robotHeight_cm

        correction_factor = delta_f_cm/cameraHeight_cm
        #print(f'correction_factor: {correction_factor}')

        corrected_r = r * correction_factor
        #print(f'corrected_r: {corrected_r}')

        corrected_x = cameraCoordinates[0]+(x_diff/r)*corrected_r
        corrected_y = cameraCoordinates[1]+(y_diff/r)*corrected_r

        #print(f'correctionValue_x: {(x_diff/r)*correction_factor}')
        #print(f'correctionValue_y: {(y_diff/r)*correction_factor}')

        return corrected_x, corrected_y
    
    else: 
        return robotCoordinates[0],robotCoordinates[1]

