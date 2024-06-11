import cv2
import numpy as np
from src.Components.BallDetection import DetectBall
from src.Components.EggDetection import DetectEgg
from src.Components.RobotDetection import DetectRobot

def significant_change(new, old, threshold=20):
    if new is None or old is None:
        return True
    return np.linalg.norm(new - old) > threshold

def infiniteCapture(results, threshold=20):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Unable to open the camera.")
        results["error"] = "Unable to open camera"
        return
    
    last_balls = None
    last_egg = None
    last_robot_contour = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture frame.")
            results["error"] = "Unable to capture frame"
            break

        balls = DetectBall(frame)
        egg = DetectEgg(frame)
        robot_contour = DetectRobot(frame)

        if balls is not None:
            balls = np.uint16(np.around(balls))
            if significant_change(balls, last_balls, threshold):
                results["balls"] = balls
                last_balls = balls

        if egg is not None:
            egg = np.uint16(np.around(egg))
            if significant_change(egg, last_egg, threshold):
                results["egg"] = egg
                last_egg = egg

        if robot_contour is not None:
            if significant_change(robot_contour, last_robot_contour, threshold):
                results["robot_contour"] = robot_contour
                last_robot_contour = robot_contour

    cap.release()
    cv2.destroyAllWindows()