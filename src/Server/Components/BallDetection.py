import os
import cv2
import numpy as np

from src.Server.Components.CourseDetection import detect_color, convert_to_binary

def DetectBall(frame):
    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use Hough Circle Transform to detect circles. These values have been modified to fit ball detection FHD resolution
    # Changing the visual size as well as resolution of picture of the balls will affect the circle detection
    # Needs camera distance calibration to find the proper values
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT_ALT,
        dp=1,
        minDist=20,
        param1=50,
        param2=0.9,
        minRadius=10,
        maxRadius=20
    )
    if circles is not None:
        return circles


def DetectOrangeBall(frame):
    #lower_orange = np.array([10, 20, 70])
    #upper_orange = np.array([30, 255, 255])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #mask = cv2.inRange(hsv, lower_orange, upper_orange)
    mask = convert_to_binary(hsv)
    #mask = cv2.GaussianBlur(mask, (7, 7), 2)
    cv2.imshow("mask", mask)

    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT_ALT,
        dp=1.5,
        minDist=20,
        param1=50,
        param2=0.9,
        minRadius=10,
        maxRadius=20
    )
    if circles is not None:
        return circles
