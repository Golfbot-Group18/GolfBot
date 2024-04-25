import os
import cv2
import numpy as np
# Get the current directory where your script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Provide the correct full path to the image file
image_path = os.path.join(script_dir, '..', 'Data', 'Images', 'test1.jpg')
# Load the image
img = cv2.imread(image_path)


# Function to detect red areas in the image
def detect_red(image):
    # Define the lower and upper bounds for the red color in HSV color space
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Apply morphological operations to remove noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours on the original image
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:  # Adjust the area threshold as needed
            cv2.drawContours(image, [contour], -1, (0, 0, 255), 2)

    return image


# Check if the image is loaded successfully
if img is None:
    print(f"Error: Unable to load the image from '{image_path}'.")
else:
    # Detect red areas in the image
    course = detect_red(img)
    cv2.namedWindow('Objects Detected', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Objects Detected', 720, 1280)
    cv2.imshow('Objects Detected', course)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
