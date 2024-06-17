import math
import DetectionMethods as detectionMethods
import cv2
import numpy as np


def DetectRobot(frame):
    lower_green = np.array([50, 45, 80])
    upper_green = np.array([100, 150, 255])

    # lower_blue = np.array([0, 60, 90])
    # upper_blue = np.array([255, 100, 100])

    green_area = detectionMethods.DetectColor(frame, lower_green, upper_green)
    # blue_area = DetectColor(frame, lower_blue, upper_blue)
    # tip =  DetectRobotEdge(frame)

    return green_area, None


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
        raise ValueError("The approximated contour does not represent a triangle")

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
    sorted_lengths = CalculateRobotTriangle(contour)
    shortest_length = sorted_lengths[0]

    return shortest_length[0] * 2


def CalculateRobotHeading(contour):
    # Returns coordinates for tip of the robot
    sorted_lengths = CalculateRobotTriangle(contour)
    # side_length_1 = sorted_lengths[0]
    side_length_2 = sorted_lengths[1]
    side_length_3 = sorted_lengths[2]

    pt1, pt2 = side_length_3[1]
    pt3, pt4 = side_length_2[1]

    overlapping_point = 0

    if np.array_equal(pt1, pt3) or np.array_equal(pt1, pt4):
        overlapping_point = pt1
    elif np.array_equal(pt2, pt3) or np.array_equal(pt2, pt4):
        overlapping_point = pt2
    # Make it return the other point from side a so that we have a direction
    return overlapping_point


"""""
# Får den aktuelle mappe, hvor vores script ligger og den korrekte stig til billede filerne
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, '..', 'data', 'images', 'Robot_green.jpg')
img = cv2.imread(image_path)

x, y, w, h = RobotDetection(img)
# Draw the bounding rectangle on the original image
cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle
cv2.namedWindow('Course Detected', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Course Detected', 720, 1280)
cv2.imshow('Course Detected', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""""
