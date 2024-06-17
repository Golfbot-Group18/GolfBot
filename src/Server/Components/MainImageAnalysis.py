import cv2
import numpy as np

from src.Server.Camera.Calibration import CalibrateCamera
from src.Server.Components.BallDetection import *
from src.Server.Components.EggDetection  import *
from src.Server.Components.RobotDetection  import *


def infiniteCapture():
    # Open a connection to the camera (0 is usually the default camera)
    cap = cv2.VideoCapture(0)

    #For windows
    #cap = cv2.VideoCapture(1)
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Unable to capture frame.")
            break
        else:
            frame = CalibrateCamera(frame)
            balls = DetectBall(frame)
            eggs = DetectEgg(frame)
            green_area, blue_area = DetectRobot(frame)
            orange_ball = DetectOrangeBall(frame)

            # Draw the bounding rectangle on the original image
            if green_area is not None:
                cv2.drawContours(frame, [green_area], -1, (0, 255, 0), 2)
                for contour in green_area:
                    for point in contour:
                        x, y = point
                        print(f"Green point: ({x}, {y})")

            if blue_area is not None:
                cv2.drawContours(frame, [blue_area], -1, (0, 255, 0), 2)
                for contour in blue_area:
                    for point in contour:
                        x, y = point
                        print(f"Blue point: ({x}, {y})")
            # If balls are found, draw them on the image
            if balls is not None:
                balls = np.uint16(np.around(balls))

                for i, circle in enumerate(balls[0, :]):
                    # Draw the outer circle
                    cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                    # Draw the center of the circle
                    cv2.circle(frame, (circle[0], circle[1]), 2, (0, 0, 255), 3)

                    label_position = (circle[0] - 10, circle[1] - 10)
                    cv2.putText(frame, "ball", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # If eggs are found, draw them on the image
            if eggs is not None:
                for contour in eggs:
                    cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)  # Red color

            # Display the frame with circles and labels

            # If orange ball is found, draw them on the image
            if orange_ball is not None:
                for contour in orange_ball:
                    cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)  # Red color

            #Also for windows
            #cv2.namedWindow('Objects Detected', cv2.WINDOW_NORMAL)
            #cv2.resizeWindow('Objects Detected', 1280, 720)
            cv2.imshow('Objects Detected', frame)

            # Break the loop if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()
    return balls, eggs, green_area

def giveMeFrames():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to capture frame.")
    else:
        return frame

def bitchImagePlease():
    # Open a connection to the camera (0 is usually the default camera)
    cap = cv2.VideoCapture(0)
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Unable to capture frame.")
        #break
    else:
        balls = DetectBall(frame)
        egg = DetectEgg(frame)
        robot_contour = DetectRobot(frame)
        # Draw the bounding rectangle on the original image
        if robot_contour is not None:
            cv2.drawContours(frame, [robot_contour], -1, (0, 255, 0), 2)
            for contour in robot_contour:
                for point in contour:
                    x, y = point
                    #print(f"Green point: ({x}, {y})")


        # If balls are found, draw them on the image
        if balls is not None:
            balls = np.uint16(np.around(balls))

            for i, circle in enumerate(balls[0, :]):
                # Draw the outer circle
                cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                # Draw the center of the circle
                cv2.circle(frame, (circle[0], circle[1]), 2, (0, 0, 255), 3)

                label_position = (circle[0] - 10, circle[1] - 10)
                cv2.putText(frame, "ball", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # If eggs are found, draw them on the image
        if egg is not None:
            egg = np.uint16(np.around(egg))

            for i, circle in enumerate(egg[0, :]):
                # Draw the outer circle
                cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                # Draw the center of the circle
                cv2.circle(frame, (circle[0], circle[1]), 2, (0, 0, 255), 3)

                label_position = (circle[0] - 10, circle[1] - 10)
                cv2.putText(frame, "Egg", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the frame with circles and labels
        cv2.imshow('Objects Detected', frame)

        # Break the loop if 'q' key is pressed
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()
    
    return balls, egg, robot_contour