import os
import cv2
import numpy as np
import src.Server.Components.DetectionMethods as detectionMethods
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


def OldDetectOrangeBall(frame):
    lower_orange = np.array([10, 20, 70])
    upper_orange = np.array([30, 255, 255])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    # mask = convert_to_binary(hsv)
    # mask = cv2.GaussianBlur(mask, (7, 7), 2)
    orange_ball, canny = detectionMethods.DetectEllipse(frame, (10, 10), (20, 20), 1, 200)
    cv2.imshow("Canny Image", canny)

    cv2.imshow("mask", mask)

    return None


def DetectOrangeBall(frame):
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([15, 255, 255])
    min_area = 300
    max_area = 1000  # Adjust based on your specific case

    orange_ball = detectionMethods.DetectBallContour(frame, min_area, max_area, lower_orange, upper_orange)


    return orange_ball



