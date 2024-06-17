import os
import cv2
import numpy as np

from src.Server.Camera.Calibration import CalibrateCamera
from src.Server.Components.BallDetection import *
from src.Server.Components.EggDetection import *
from src.Server.Components.RobotDetection import *
from src.Server.Components.DetectionMethods import *

#just a change to test git
# Get the current directory where your script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Provide the correct full path to the image file
image_path = os.path.join(script_dir, '..', 'Images', 'Robot_green.jpg')

# image_path = os.path.join(os.getcwd(), 'Images', 'test1.jpg')
# Load the image
frame = cv2.imread(image_path)

# Check if the image is loaded successfully
if frame is None:
    print(f"Error: Unable to load the image from '{image_path}'.")
else:
    new_frame = CalibrateCamera(frame)

    orange = None
    balls, orange_index = DetectBalls(frame)
    egg = DetectEgg(frame)
    green_area, blue_area = DetectRobot(frame)

    eggs = DetectEgg(frame)
    orange_ball = DetectOrangeBall(frame)

    # im_with_keypoints = cv2.drawKeypoints(frame, eggs, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # rect = cv2.minAreaRect(green_area)
    #print(rect)
    #box = cv2.boxPoints(rect)
    #box = np.int32(box)
    #cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

    '''
    epsilon = 0.02 * cv2.arcLength(green_area, True)
    approx = cv2.approxPolyDP(green_area, epsilon, True)
    print(approx)
    cv2.drawContours(frame, [approx], 0, (0, 0, 255), 2)

    robot_width = CalculateRobotWidth(green_area)
    print(f"Robot width: {robot_width}")

    robot_heading = CalculateRobotHeading(green_area)
    print(f"Heading coordinate: {robot_heading}")
    '''


    #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle


    # Draw the bounding rectangle on the original image
    if green_area is not None:
        #cv2.drawContours(frame, [green_area], -1, (0, 255, 0), 2)
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
    if orange is not None:
        balls = np.uint16(np.around(balls))
        print("Orange ball Coordinates:")
        for i, circle in enumerate(balls[0, :]):
            # Draw the outer circle
            cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(frame, (circle[0], circle[1]), 2, (0, 0, 255), 3)

            label_position = (circle[0] - 10, circle[1] - 10)
            cv2.putText(frame, "orange ball", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            x, y, radius = circle
            print(f"Center coordinates: ({x}, {y}), Radius: {radius}")



    # If balls are found, draw them on the image
    if balls is not None:
        balls = np.uint16(np.around(balls))
        print("Ball Coordinates:")
        for i, circle in enumerate(balls[0, :]):
            # Draw the outer circle
            cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(frame, (circle[0], circle[1]), 2, (0, 0, 255), 3)

            label_position = (circle[0] - 10, circle[1] - 10)
            if i == orange_index:
                cv2.putText(frame, "orange ball", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                x, y, radius = circle
                print(f"Orange Ball center coordinates: ({x}, {y}), Radius: {radius}")
            else:
                cv2.putText(frame, "ball", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                x, y, radius = circle
                print(f"Center coordinates: ({x}, {y}), Radius: {radius}")

    # If eggs are found, draw them on the image
    if eggs is not None:
        for contour in eggs:
            cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)  # Red color

    # If orange ball is found, draw them on the image
    if orange_ball is not None:
        for contour in orange_ball:
            cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)  # Red color


    # Display the image with detected circles and contours
    cv2.imshow('Objects Detected', frame)
  #  cv2.imshow('Blobs Detected', im_with_keypoints)
    # cv2.imshow('Undistorted', new_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()