import cv2
import numpy as np
import src.Server.Components.DetectionMethods as detectionMethods


def DetectAllBalls(frame, isolate_white_balls=False, minimum_distance_between_balls=15, perfect_circle_deviation=0.8, min_radius=10, max_radius=20):
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
        minDist=minimum_distance_between_balls,
        param1=50,
        param2=perfect_circle_deviation,
        minRadius=min_radius,
        maxRadius=max_radius
    )
    if circles is not None:
        return circles
    else:
        return None


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
    upper_orange = np.array([25, 255, 255])

    # lower_orange = np.array([5, 100, 100])
    # upper_orange = np.array([28, 255, 255])

    min_area = 300
    max_area = 1000  # Adjust based on your specific case

    orange_ball = detectionMethods.DetectBallContour(frame, min_area, max_area, lower_orange, upper_orange)

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


def PopBallFromTuple(balls, ball_to_pop):
    balls_list = list(balls[0])
    balls_list.pop(ball_to_pop)
    return tuple(balls, )

def GetFixedBallPoints():
    return [[[1313, 539, 15.52368]]]


def is_point_in_proximity(point, tree, threshold=1.5):
    result = tree.query_ball_point(point, threshold)
    return len(result) > 0
