import os
import cv2
import numpy as np
from scipy.spatial import KDTree
from Components.BallDetection import DetectOrangeBall
from sklearn.cluster import DBSCAN


# Får den aktuelle mappe, hvor vores script ligger og den korrekte sti til billede filerne
# script_dir = os.path.dirname(os.path.abspath(__file__))
# image_path = os.path.join(script_dir, '..', 'Data', 'Images', 'Empty_course.jpeg')
# img = cv2.imread(image_path)

def giveMeBinaryBitch(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = cv2.bitwise_or(mask1, mask2)

    _, binary_image = cv2.threshold(mask, 128, 255, cv2.THRESH_BINARY)

    # Subtract the orange ball
    orange_ball = DetectOrangeBall(img)
    binary_contour_mask = np.zeros_like(binary_image)
    result_image = binary_image

    if orange_ball is not None:
        for contour in orange_ball:
            cv2.drawContours(binary_contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
            dilation_kernel = np.ones((5, 5), np.uint8)
            expanded_mask = cv2.dilate(binary_contour_mask, dilation_kernel, iterations=1)
            result_image = cv2.subtract(binary_image, expanded_mask)

    cv2.imshow('Result without orange ball subtraction', binary_image)

    return result_image


def giveMeCourseFramePoints(img):
    result_image = giveMeBinaryBitch(img)
    corners = cv2.goodFeaturesToTrack(result_image, maxCorners=100, qualityLevel=0.27, minDistance=10, blockSize=5,
                                      useHarrisDetector=True)

    if corners is not None:
        corners = np.int32(corners)
        corners = np.array([corner.ravel() for corner in corners])

        # Perform DBSCAN clustering
        clustering = DBSCAN(eps=30, min_samples=4).fit(corners)
        labels = clustering.labels_

        # Extract unique clusters
        unique_labels = set(labels)

        # Find the centroids of the clusters
        centroids = []
        for label in unique_labels:
            cluster_points = corners[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)

        # Ensure we have exactly four points
        # if len(centroids) != 4:
        #    raise ValueError("Expected to find exactly four intersection points")

        # Convert centroids to integer points if needed
        centroids = np.int32(centroids)

        # Now `centroids` contains the four intersection points
        # print("Intersection Points:", centroids)
        hull = cv2.convexHull(centroids)
        epsilon = 0.1 * cv2.arcLength(hull, True)
        approximated_points = cv2.approxPolyDP(hull, epsilon, True)

        def sort_vertices(vertices):
            vertices = vertices.reshape((4, 2))
            sorted_vertices = sorted(vertices, key=lambda y: y[0])
            left_points = sorted(sorted_vertices[:2], key=lambda y: y[1])
            right_points = sorted(sorted_vertices[2:], key=lambda y: y[1])
            return np.array([left_points[0], right_points[0], right_points[1], left_points[1]], dtype='int32')

        sorted_points = sort_vertices(approximated_points)

        # for i, point in enumerate(sorted_points):
        #    cx, cy = point
        #    cv2.putText(img, f'({cx}, {cy})', (cx + 5, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0),1)

        # cv2.drawContours(img, [sorted_points], 0, (255, 0, 0), 2)
        # print("Printing approximated points:", sorted_points)
        # cv2.imshow('Result', img)

        # Draw the corners on the result image
        # for corner in corners:
        #     x, y = corner.ravel()
        #     cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

        # for centroid in centroids:
        #     x, y = centroid
        #     cv2.circle(img, (x, y), 5, (0, 255, 0), -1)  # Red circles for centroids

        # cv2.drawContours(img, [approximated_points], 0, (255, 0, 0), 2)
        # print("Printing approximated points:", approximated_points)
        # cv2.imshow('Result', img)

        # The returned points are: Upper left, Upper Right, Lower Right, Lower Left
        return sorted_points
    else:
        return None


def giveMeGoalPoints(frame):
    points = giveMeCourseFramePoints(frame)
    if points is not None:
        top_left = points[0]
        top_right = points[1]
        bottom_right = points[2]
        bottom_left = points[3]

        print("Printing points:")
        print(top_left, top_right, bottom_right, bottom_left)

        # ts and tb is the coefficient for how much the point is shifted to account for the goals
        # being in the middle but slightly deviated from it, meaning it's not dead center

        ts = 1 - 0.512  # percentage of length of the side length where center point is, for small goal- 51.2% of 125 cm
        tb = 1 - 0.5  # same but big goal 50% of 125cm
        small_goal_center_point = (
            (1 - ts) * bottom_right[0] + ts * top_right[0],
            (1 - ts) * bottom_right[1] + ts * top_right[1]
        )

        # If you also need to calculate for bottom_left and top_left at 51.2%
        big_goal_center_point = (
            (1 - tb) * bottom_left[0] + tb * top_left[0],
            (1 - tb) * bottom_left[1] + tb * top_left[1]
        )

        return small_goal_center_point, big_goal_center_point
    else:
        return None


def detect_color(img, color_range):
    mask = cv2.inRange(img, color_range[0], color_range[1])
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 2)
    return img, mask


def convert_to_binary(img, threshold_value=128):
    # Konverter billedet til gråskala
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Anvend binær tærskelværdi
    _, binary_image = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY)
    return binary_image


def generate_kd_tree(course_coords):
    return KDTree(course_coords)


'''Deprecated'''


def course_coordinates(binary_image):
    white_coordinates = np.argwhere(binary_image == 255)

    # Print the coordinates
    for coord in white_coordinates:
        print(f"White pixel at (x, y): ({coord[1]}, {coord[0]})")


'''Deprecated'''


def define_inner_frame(mask, img):
    # Finder kanterne på objekter i en mask og tegner om dem
    x_min, x_max, y_min, y_max = 0, 1800, 0, 1200
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 5000:  # Område tærskel for at identificere signifikante objekter
            x, y, w, h = cv2.boundingRect(contour)
            if x > x_min and x + w < x_max and y > y_min and y + h < y_max:
                cv2.drawContours(img, [contour], 0, (0, 0, 255), 9)
    return img


'''Deprecated'''


def draw_coordinate_system(img, interval=100, color_x=(255, 0, 0), color_y=(0, 255, 0)):
    # Tegner koordinatsystem på billedet
    height, width = img.shape[:2]
    # X-akse
    for x in range(0, width, interval):
        cv2.line(img, (x, 0), (x, height), color_x, 1)
        cv2.putText(img, str(x), (x, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_x, 1)
    # Y-akse
    for y in range(0, height, interval):
        cv2.line(img, (0, y), (width, y), color_y, 1)
        cv2.putText(img, str(y), (width // 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_y, 1)


'''
# Verifikation af at billede læses korrekt
if img is None:
    print(f"Fejl: Kan ikke indlæse billedet fra '{image_path}'.")
else:
    # Farvedetektion
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_lower = np.array([0, 50, 50])  # Lavere HSV grænse for rød
    red_upper = np.array([10, 255, 255])  # Øvre HSV grænse for rød
    img_with_red_contours, red_mask = detect_color(hsv_image, (red_lower, red_upper))

    # Anvend define_inner_frame og draw_coordinate_system
    path_img = define_inner_frame(red_mask, img)
    ##draw_coordinate_system(path_img, interval=100, color_x=(255, 0, 0), color_y=(0, 255, 0))

    # Konverter fra BGR til HSV farverummet
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Konverter HSV-billede til binært billede
    binary_image = convert_to_binary(hsv)

    # Generer gitter af 0'er og 1'ere
    grid = generate_grid(binary_image, interval=1)
    for row in grid:
        print(row)
    
    grid_image = visualize_grid(grid, interval=10)
    cv2.imshow('Grid Visualization', grid_image)

    course_coordinates(binary_image)

    # Vis det færdige resultat
    cv2.namedWindow('Course Detected', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Course Detected', 720, 1280)
    cv2.imshow('Course Detected', path_img)
    cv2.imshow('Binary Image', binary_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
'''
