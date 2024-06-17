import os
import cv2
import numpy as np
import DetectionMethods as detectionMethods
def OldDetectEgg(frame):
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
        param2=0.7,
        minRadius=25,
        maxRadius=30
    )
    if circles is not None:
        return circles


def DetectEgg(frame):
    min_canny_threshold = 100
    max_canny_threshold = 200
    min_ellipse_size = (30, 30)  # Minimum width and height of the ellipse
    max_ellipse_size = (100, 100)  # Maximum width and height of the ellipse
    eggs = detectionMethods.DetectEllipse(frame, min_ellipse_size, max_ellipse_size, min_canny_threshold, max_canny_threshold)
    return eggs
