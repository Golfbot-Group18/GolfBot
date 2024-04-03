import cv2
import numpy as np

# Provide the correct full path to the image file
image_path = '/Users/hallahalhag/Desktop/Project software /image detaction/Annotating_images_using_OpenCV/python/app/datasets/imags/Banen1.jpg'

# Load the image
img = cv2.imread(image_path)

# Check if the image is loaded successfully
if img is None:
    print(f"Error: Unable to load the image from '{image_path}'.")
else:
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help the circle detection
    gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use Hough Circle Transform to detect circles
    circles = cv2.HoughCircles(
        gray_blurred,
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


        # Display the image with circles
        cv2.imshow('Circles Detected', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No circles detected in the image.")