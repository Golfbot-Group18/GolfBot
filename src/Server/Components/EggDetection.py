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


def DetectEggOld(frame):
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


def DetectEgg(frame):
    min_canny_threshold = 100
    max_canny_threshold = 200
    min_ellipse_size = (30, 30)  # Minimum width and height of the ellipse
    max_ellipse_size = (100, 100)  # Maximum width and height of the ellipse

    grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(grey_image, 230, 255, cv2.THRESH_BINARY)
    inverted_image = cv2.bitwise_not(binary_image)

    # Gaussian blur
    img_size = (9, 9)
    gaussian_image = cv2.GaussianBlur(inverted_image, img_size, 4)

    # Canny edge detector
    canny_image = cv2.Canny(gaussian_image, min_canny_threshold, max_canny_threshold)

    # Find contours
    contours, _ = cv2.findContours(canny_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Fit ellipses to the contours
    ellipse_contours = []
    for contour in contours:
        if len(contour) >= 5:  # FitEllipse needs at least 5 points
            ellipse = cv2.fitEllipse(contour)
            width, height = ellipse[1]
            if min_ellipse_size[0] <= width <= max_ellipse_size[0] and min_ellipse_size[1] <= height <= \
                    max_ellipse_size[1]:
                ellipse_contour = cv2.ellipse2Poly((int(ellipse[0][0]), int(ellipse[0][1])),
                                                   (int(width / 2), int(height / 2)),
                                                   int(ellipse[2]), 0, 360, 10)
                ellipse_contours.append(ellipse_contour)
    return ellipse_contours
