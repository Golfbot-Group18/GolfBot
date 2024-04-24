import os
import cv2
import numpy as np
#just a change to test git
# Get the current directory where your script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Provide the correct full path to the image file
image_path = os.path.join(script_dir, '..', 'Data', 'Images', 'test1.jpg')

# image_path = os.path.join(os.getcwd(), 'Images', 'test1.jpg')
# Load the image
img = cv2.imread(image_path)

# Check if the image is loaded successfully
if img is None:
    print(f"Error: Unable to load the image from '{image_path}'.")
else:
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help the circle detection
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    cv2.imshow('Gaussian Blur', blurred)
    cv2.waitKey(0)

    # Use Hough Circle Transform to detect circles for the ball
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=50,
        param1=50,
        param2=30,
        minRadius=10,
        maxRadius=50
    )

    # If circles are found for the ball, draw them on the image
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i, circle in enumerate(circles[0, :]):
            # Draw the outer circle
            cv2.circle(img, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(img, (circle[0], circle[1]), 2, (0, 0, 255), 3)
            label_position = (circle[0] - 10, circle[1] - 10)
            cv2.putText(img, "golf_ball", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Threshold the image to segment the white cork (adjust the threshold value as needed)
    _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # Find contours in the thresholded image for the egg
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    """""
    # Iterate through detected contours for the egg
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # If the contour has a reasonable area (adjust threshold as needed)
        if area > 1000:
            # Draw the contour on the original image
            cv2.drawContours(img, [contour], -1, (255, 0, 0), 2)

            # Label the contour as "egg"
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                label_position = (cX - 10, cY - 10)
                cv2.putText(img, "egg", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
"""""



    # Display the image with detected circles and contours
    cv2.imshow('Objects Detected', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


"""""
import cv2

import numpy as np

# Provide the correct full path to the image file
image_path = '/Users/hallahalhag/Documents/GitHub/GolfBot.Group18/src/app/datasets/Images/test1.jpg'

# Load the image
img = cv2.imread(image_path)

# Check if the image is loaded successfully
if img is None:
 print(f"Error: Unable to load the image from '{image_path}'.")
else:
    ball_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(ball_gray, (9, 9), 2)

    # Use Hough Circle Transform to detect circles
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=50,
        param1=50,
        param2=30,
        minRadius=10,
        maxRadius=50
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
            cv2.putText(img, "ball", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # Find contours in the grayscale image
    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through detected contours
    for contour in contours:
         # Approximate the contour to a polygon
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            # If the contour has a reasonable area (adjust threshold as needed)
            if cv2.contourArea(contour) > 1000:
                # Draw the contour on the image
                cv2.drawContours(img, [approx], 0, (255, 0, 0), 2)

                label_position = (approx[0][0][0], approx[0][0][1])

    cv2.putText(img, "egg", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    egg_circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=50,
        param1=50,
        param2=30,
        minRadius=10,
        maxRadius=50
    )

        # Display the image with circles
    cv2.imshow('Objects Detected', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
       # else:
      #  print("No circles detected in the image.")
      """