import os
import cv2
import numpy as np

'''
# Get the current directory where your script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Provide the correct full path to the image file
image_path = os.path.join(script_dir, '..', 'Data', 'Images', 'Robot_green2.jpg')
# Load the image
img = cv2.imread(image_path)


# Check if the image is loaded successfully
if img is None:
    print(f"Error: Unable to load the image from '{image_path}'.")
else:

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use Hough Circle Transform to detect circles. These values have been modified to fit ball detection FHD resolution
    # Changing the visual size as well as resolution of picture of the balls will affect the circle detection
    # Needs camera distance calibration to find the proper values
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT_ALT,
        dp=1,
        minDist=20,
        param1=50,
        param2=0.7,
        minRadius=20,
        maxRadius=25
    )

    # If circles are found, draw them on the image
    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i, circle in enumerate(circles[0, :]):
            # Draw the outer circle
            cv2.circle(img, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(img, (circle[0], circle[1]), 2, (0, 0, 255), 3)

            label_position = (circle[0] - 10, circle[1] - 10)
            cv2.putText(img, "Egg", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Display the image with circles and detected red areas
    cv2.namedWindow('Eggs Detected', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Eggs Detected', 720, 1280)
    cv2.imshow('Eggs Detected',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

'''


def DetectEgg(frame):
    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use Hough Circle Transform to detect circles. These values have been modified to fit ball detection FHD resolution
    # Changing the visual size as well as resolution of picture of the balls will affect the circle detection
    # Needs camera distance calibration to find the proper values
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT_ALT,
        dp=1,
        minDist=20,
        param1=50,
        param2=0.7,
        minRadius=25,
        maxRadius=30
    )
    if circles is not None:
        return circles


def DetectEgg2(frame):
    min_threshold = 100  # Adjust based on your requirements
    max_threshold = 200  # Adjust based on your requirements

    # Read the image in grayscale
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Invert the image
    img = cv2.bitwise_not(img)

    # Apply Gaussian blur
    img_size = (9, 9)
    img = cv2.GaussianBlur(img, img_size, 4)

    # Apply Canny edge detector
    img_canny = cv2.Canny(img, min_threshold, max_threshold)

    # Find contours
    contours, hierarchy = cv2.findContours(img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Fit ellipses to the contours
    rectangles = []
    for contour in contours:
        if len(contour) >= 5:  # FitEllipse needs at least 5 points
            ellipse = cv2.fitEllipse(contour)
            rectangles.append(ellipse)

    # Convert the image to BGR
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Draw ellipses on the image
    for rectangle in rectangles:
        cv2.ellipse(img_color, rectangle, (0, 0, 255), 2)  # Red color

    # Display the image with ellipses
    cv2.imshow('Ellipses', img_color)

    return None
