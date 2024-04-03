import cv2

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

    # Apply GaussianBlur to reduce noise and help the rectangle detection
    gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use Hough Rectangle Transform to detect rectangles (contours)
    contours, _ = cv2.findContours(gray_blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If contours are found, draw rectangles around them
    if contours:
        for contour in contours:
            # Approximate the contour to a rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Draw the rectangle
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Label the 
            # name "ball9"
            label_position = (x - 10, y - 10)
            cv2.putText(frame, "ball", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Display the frame with rectangles and labels
        cv2.imshow('Rectangles Detected', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows() 
