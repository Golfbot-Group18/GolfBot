import cv2
import numpy as np

# Open a connection to the camera (0 is usually the default camera)
cap = cv2.VideoCapture(0)
num = 9
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Unable to capture frame.")
        break
    # Display the frame with circles and labels
    cv2.imshow('Camera View', frame)

    if cv2.waitKey(5) & 0xFF == ord('s'):
        cv2.imwrite('RobotTriangle' + str(num) + '.png', frame)
        print("image saved!")
        num += 1



    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
