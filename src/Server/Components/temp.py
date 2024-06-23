import cv2
import numpy as np
from RobotDetection import DetectRobot, CalculateRobotHeading
from EggDetection import DetectEgg
from CourseDetection import *
from GridGeneration import *


img = cv2.imread("src\Server\Components\Robot_green.jpg")


#cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
# Capture frame-by-frame

#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
#ret, img = cap.read()





'''
robot_contour = DetectRobot(img)
if robot_contour is not None: 
    cv2.drawContours(img, [robot_contour],-1, (0,255,0), 2)    
robot_heading_points = CalculateRobotHeading(robot_contour)
print(f'Robot heading: {robot_heading_points}')
robotCoordinates = (robot_heading_points[0][0], robot_heading_points[0][1])
print(robotCoordinates)
x, y = realCoordinates(robotHeight=21.3, cameraHeight=161, robotCoordinates=robotCoordinates)
print(f'x: {x}, y: {y}')

'''
eggContour = DetectEgg(img)

binary_picture = giveMeBinaryBitch(img)
standard_grid = generate_grid(binary_picture, interval=1)

obstacle_coords = find_obstacle_coords(standard_grid)
obstacle_grid = create_obstacle_grid(obstacle_coords, standard_grid.shape)
obstacle_grid = add_contour_to_obstacle_grid(eggContour,obstacle_grid)

grid_with_egg = visualize_grid(obstacle_grid)

cv2.imshow("Grid w egg", grid_with_egg)
cv2.imshow('Objects Detected', img)

cv2.namedWindow('Grid w egg', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Grid w egg', 1920, 1080)
cv2.waitKey(1) 




while(1):
    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
