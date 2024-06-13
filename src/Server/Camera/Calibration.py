import cv2 as cv
import pickle
import os

# Get the current directory where your script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Provide the correct full path to the image file
camera_matrix_path = os.path.join(script_dir, 'cameraMatrix.pkl')
dist_path = os.path.join(script_dir, 'dist.pkl')

def CalibrateCamera(frame):
    camera_matrix = pickle.load(open(camera_matrix_path, "rb"))
    dist = pickle.load(open(dist_path, "rb"))

    undistorted_frame = cv.undistort(frame, camera_matrix, dist, None, camera_matrix)

    return undistorted_frame
