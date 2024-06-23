import math
import cv2
import numpy as np
from src.Server.Components.DetectionMethods import DetectColor


def DetectRobot(frame):
    lower_green = np.array([50, 45, 80])
    upper_green = np.array([100, 150, 255])

    # lower_blue = np.array([0, 60, 90])
    # upper_blue = np.array([255, 100, 100])

    green_area = DetectColor(frame, lower_green, upper_green)
    if green_area is not None:
        return green_area
    else:
        return None


def euclidean_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def CalculateRobotTriangle(contour):
    # This function only works for a right-angled triangle on the robot
    # with precisely half the width of it in the middle

    # Creates a minimum area rectangle that wraps around
    # rect = cv2.minAreaRect(contour)
    # width, height = rect[1]

    # Approximates and straightens the contour lines so that it extracts the vertices of the shape
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approximated_points = cv2.approxPolyDP(contour, epsilon, True)

    # If the approximation returns a list longer than 3,
    # it means that the shape is not a triangle as it has more than 3 sides
    if len(approximated_points) != 3:
        return None
    else:
        # Reshapes the way the data is stored
        points = approximated_points.reshape((3, 2))
        pt1, pt2, pt3 = points[0], points[1], points[2]

        side_length_1 = euclidean_distance(pt1, pt2), (pt1, pt2)
        side_length_2 = euclidean_distance(pt1, pt3), (pt1, pt3)
        side_length_3 = euclidean_distance(pt2, pt3), (pt2, pt3)

        # Find the index and value of the hypotenuse (the longest side)
        lengths = [side_length_1, side_length_2, side_length_3]
        sorted_lengths = sorted(lengths, key=lambda x: x[0])

        return sorted_lengths


def CalculateRobotWidth(contour):
    if contour is not None:
        sorted_lengths = CalculateRobotTriangle(contour)
        if sorted_lengths is not None:
            shortest_length = sorted_lengths[0]
            return shortest_length[0] * 2
    else:
        return None


def CalculateRobotHeading(contour):
    # Returns coordinates for tip of the robot
    sorted_lengths = CalculateRobotTriangle(contour)
    if sorted_lengths is not None:
        # side_length_1 = sorted_lengths[0]

        # Side length 2 is the adjacent side to the hypotenuse
        side_length_2 = sorted_lengths[1]
        # Side length 3 is the hypotenuse
        side_length_3 = sorted_lengths[2]

        # pt variables are unsorted points pt = (x,y) of the side lengths
        pt1, pt2 = side_length_3[1]
        pt3, pt4 = side_length_2[1]

        # Overlapping point is the point of overlap of the hypotenuse and adjacent

        if np.array_equal(pt3, pt1) or np.array_equal(pt3, pt2):
            overlapping_point = pt3
            starting_point = pt4

        elif np.array_equal(pt4, pt1) or np.array_equal(pt4, pt2):
            overlapping_point = pt4
            starting_point = pt3
        # Make it return the other point from side a so that we have a direction
        else:
            raise ValueError("Could not find overlapping point")

        return starting_point, overlapping_point
    else:
        print("Could not calculate Robot Triangle")
        return None


def Return_robot_position(frame):
    robot_contour = DetectRobot(frame)
    if robot_contour is not None:
        robot_position_points = CalculateRobotHeading(robot_contour)
        if robot_position_points is not None:
            robot_base_position = np.array(robot_position_points[0], dtype=int)
            robot_tip_position = np.array(robot_position_points[1], dtype=int)
            return robot_base_position, robot_tip_position
        else:
            print("Could not calculate robot heading")
            return None, None
    else:
        print("Could not find robot contour")
        return None, None


def Return_robot_heading(robot_base, robot_tip):
    x_c, y_c = robot_base
    x_t, y_t = robot_tip
    
    delta_x = x_t - x_c
    delta_y = y_t - y_c
    
    theta = math.atan2(delta_y, delta_x)
    
    degrees = math.degrees(theta)

    return (degrees + 180) % 360 - 180
