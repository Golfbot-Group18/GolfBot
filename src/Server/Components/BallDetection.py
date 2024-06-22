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
        _, mask = cv2.threshold(mask, 195, 255, cv2.THRESH_BINARY)
        cv2.imshow('Thresh', mask)
    # Use Hough Circle Transform to detect circles. These values have been modified to fit ball detection FHD resolution
    # Changing the visual size as well as resolution of picture of the balls will affect the circle detection
    # Needs camera distance calibration to find the proper values
    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT_ALT,
        dp=1,
        minDist=20,
        param1=50,
        param2=0.7,
        minRadius=5,
        maxRadius=20
    )
    if circles is not None:
        return circles
    else:
        return None



def DetectOrangeBall(frame):
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([15, 255, 255])
    min_area = 300
    max_area = 1000  # Adjust based on your specific case

    orange_ball = DetectBallContour(frame, min_area, max_area, lower_orange, upper_orange)

    return orange_ball


def DetectBalls(frame):
    detected_orange_ball = DetectOrangeBall(frame)
    all_balls = DetectAllBalls(frame)
    actual_orange_ball = None

    orange_ball_rectangle = [cv2.boundingRect(contour) for contour in detected_orange_ball]
    if all_balls is not None:
        for rect in orange_ball_rectangle:
            x, y, w, h = rect
            rect_center = (x + w // 2, y + h // 2)
            for i, circle in enumerate(all_balls[0, :]):
                circle_center = (circle[0], circle[1])
                dist = np.sqrt((circle_center[0] - rect_center[0]) ** 2 + (circle_center[1] - rect_center[1]) ** 2)
                if dist < 30:  # Adjust distance threshold as needed
                    actual_orange_ball = circle
                    break

        white_balls = DetectAllBalls(frame, isolate_white_balls=True)
        if actual_orange_ball is not None:
            for rect in orange_ball_rectangle:
                x, y, w, h = rect
                rect_center = (x + w // 2, y + h // 2)
                for i, circle in enumerate(white_balls[0, :]):
                    circle_center = (circle[0], circle[1])
                    dist = np.sqrt((circle_center[0] - rect_center[0]) ** 2 + (circle_center[1] - rect_center[1]) ** 2)
                    if dist < 30:  # Adjust distance threshold as needed
                        white_balls_list = list(white_balls[0])
                        white_balls_list.pop(i)
                        white_balls = tuple(white_balls_list,)
                        break

        if white_balls is not None:
            if actual_orange_ball is not None:
                all_balls = (np.array(np.vstack((actual_orange_ball, white_balls[0])), dtype=np.float32),)
                return all_balls, 0
            else:
                return white_balls, 0
    else:
        return None, None




def GetFixedBallPoints():
    return [[[1313 , 539, 15.52368]]]


def is_point_in_proximity(point, tree, threshold=1.5):
    result = tree.query_ball_point(point, threshold)
    return len(result) > 0











