import os
import cv2
import numpy as np
from Components.DetectionMethods import *


def DetectAllBalls(frame, isolate_white_balls=False):
    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.GaussianBlur(gray, (9, 9), 10)
    # cv2.imshow('Gray', blurred)

    # Parameters are changed and this thresh is sothat only whitee balls are found but is a rough filtering
    if isolate_white_balls:
        _, mask = cv2.threshold(mask, 210, 255, cv2.THRESH_BINARY)
        cv2.imshow('Thresh', mask)
    # Use Hough Circle Transform to detect circles. These values have been modified to fit ball detection FHD resolution
    # Changing the visual size as well as resolution of picture of the balls will affect the circle detection
    # Needs camera distance calibration to find the proper values
    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT_ALT,
        dp=1,
        minDist=15,
        param1=50,
        param2=0.8,
        minRadius=8,
        maxRadius=20
    )
    if circles is not None:
        return circles
    else:
        return None


def DetectOrangeBall(frame):
    # Define the broader range for the color orange in HSV color space
    lower_orange = np.array([0, 150, 150])
    upper_orange = np.array([10, 255, 255])

    lower_orange1 = np.array([15, 150, 150])
    upper_orange1 = np.array([25, 255, 255])

    # Define the border range for the color yellow in HSV color space
    lower_yellow = np.array([25, 150, 150])
    upper_yellow = np.array([35, 255, 255])

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for the different shades of orange
    mask1 = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    mask2 = cv2.inRange(hsv_frame, lower_orange1, upper_orange1)
    mask3 = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

    # Combine the masks
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.bitwise_or(mask, mask3)

    # Perform morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    cv2.imshow("mask", mask)

    min_area = 300
    max_area = 1000  # Adjust based on your specific case
    orange_ball = DetectBallContour(min_area, max_area, mask)
    return orange_ball

def WhereIsTheOrangeBall(balls, orange_ball_contour):
    if balls is not None:
        orange_ball_rectangle = [cv2.boundingRect(contour) for contour in orange_ball_contour]
        index = None
        orange_ball = None
        for rect in orange_ball_rectangle:
            x, y, w, h = rect
            rect_center = (x + w // 2, y + h // 2)
            for i, circle in enumerate(balls[0, :]):
                circle_center = (circle[0], circle[1])
                dist = np.sqrt((circle_center[0] - rect_center[0]) ** 2 + (circle_center[1] - rect_center[1]) ** 2)
                if dist < 30:  # Adjust distance threshold as needed
                    index = i
                    orange_ball = circle
                    break
        if orange_ball is not None:
            return index, orange_ball
        else:
            return None, None
    else:
        return None, None


def GetFixedBallPoints():
    return [[[1377, 518, 13.41964]]]


def is_point_in_proximity(point, tree, threshold=1.5):
    result = tree.query_ball_point(point, threshold)
    return len(result) > 0
