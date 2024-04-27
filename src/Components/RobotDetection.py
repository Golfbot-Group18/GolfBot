import os
import cv2
import numpy as np

# Får den aktuelle mappe, hvor vores script ligger og den korrekte stig til billede filerne
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, '..', 'data', 'images', 'Robot_green.jpg')
img = cv2.imread(image_path)

# Konverter fra BGR til HSV farverummet
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Check if the image is loaded successfully
if img is None:
    print(f"Error: Unable to load the image from '{image_path}'.")
else:

    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])

    # Mask the image to find all green areas
    mask = cv2.inRange(img_hsv, lower_green, upper_green)

    # Returns a list of contours, and the second value is a hierarchy (which we don’t need in this case).
    # Hence, the underscore
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours by area (you can adjust the threshold)
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]

    # Get the largest contour
    largest_contour = max(filtered_contours, key=cv2.contourArea)

    x, y, w, h = cv2.boundingRect(largest_contour)

    # Draw the bounding rectangle on the original image
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle

    cv2.namedWindow('Course Detected', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Course Detected', 720, 1280)
    cv2.imshow('Course Detected', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

