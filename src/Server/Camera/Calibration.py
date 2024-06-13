import cv2 as cv
import pickle


def CalibrateCamera(frame):
    camera_matrix = pickle.load(open("cameraMatrix.pkl", "rb"))
    dist = pickle.load(open("dist.pkl", "rb"))

    undistorted_frame = cv.undistort(frame, camera_matrix, dist, None, camera_matrix)

    return undistorted_frame
