import cv2
import numpy as np

# Open a connection to the camera (0 is usually the default camera)
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Unable to capture frame.")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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

    # If circles are found, draw and label them on the frame
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i, circle in enumerate(circles[0, :]):
            # Draw the outer circle
            cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(frame, (circle[0], circle[1]), 2, (0, 0, 255), 3)
            # Label the circle with the name "ball9"
            label_position = (circle[0] - 10, circle[1] - 10)
            cv2.putText(frame, "ball9", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Display the frame with circles and labels
        cv2.imshow('Circles Detected', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
