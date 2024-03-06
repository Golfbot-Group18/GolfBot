
import cv2
import numpy as np

# Function to recognize color
def recognize_color(frame, color_lower, color_upper):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the image to get only specified color
    mask = cv2.inRange(hsv, color_lower, color_upper)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw rectangles around the identified regions
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return frame

# Capture video from the default camera (change 0 to another value if using a different camera)
cap = cv2.VideoCapture(0)

# Define color range for blue (adjust these values based on the color you want to recognize)
blue_lower = np.array([100, 50, 50])
blue_upper = np.array([130, 255, 255])

while True:
    # Read a frame from the video stream
    ret, frame = cap.read()

    # Recognize the specified color
    result_frame = recognize_color(frame, blue_lower, blue_upper)

    # Display the result
    cv2.imshow('Color Recognition', result_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()






"""
import cv2 
import numpy as np 

def blob_detect (image, hsv_min, hsv_max, blur=0, blob_params = None, search_window = None, imshow = False ):

 if blur > 0: 
     image = cv2.blur(image, (blur, blur))
     
     if imshow: 
         cv2.imshow("blur", image) 
         cv2.waitKey(0)
         
 if search_window is None: search_window = [0.0, 0.0 , 1.0, 1.0]
 
 hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
 mask = cv2.inRange(hsv, hsv_min, hsv_max)  
       
    
if imshow: 
    cv2.imshow("HSV Mask", mask)
    
mask = cv2.dilate(mask, None, iter)  

if imshow: 
    cv2.imshow("Dilate Mask", mask)
    cv2.waitKey(0)
mask = cv2.erode(mask, None, iterations=)
    
     
"""







