# THIS CODE IS TAKEN FROM GEEKS FOR GEEKS AND IS INTENDED AS TESTING CODE TO FIND POINTS
# ON THE SCREEEN WHEN CLICKS
import os

# importing the module
import cv2
import numpy as np

import src.Server.Camera.Calibration as Calibration
import src.Server.Components.CourseDetection as CourseDetection

# Provide the correct full path to the image file
image_path = "/Sandbox/Images/Refiner2.png"

hsv_values = []
# function to display the coordinates of
# of the points clicked on the image
def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Displaying the coordinates on the Shell
        print(x, ' ', y)

        # Displaying the coordinates on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f'{x},{y}', (x, y), font, 1, (255, 0, 0), 2)
        cv2.imshow('image', img)

    if event == cv2.EVENT_RBUTTONDOWN:
        # Displaying the coordinates on the Shell
        print(x, ' ', y)

        # Get the BGR color of the clicked point
        b, g, r = img[y, x]

        # Convert BGR to HSV
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = hsv_img[y, x]
        hsv_values.append((h, s, v))
        # Displaying the HSV values on the Shell
        print(f'HSV values at ({x}, {y}) -> H: {h}, S: {s}, V: {v}')

        # Displaying the coordinates and HSV values on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        cv2.putText(img, f'{b},{g},{r}', (x, y), font, 1, (255, 255, 0), 2)
        cv2.putText(img, f'HSV: {h},{s},{v}', (x, y + 30), font, 0.5, (255, 255, 0), 1)
        cv2.imshow('image', img)


    # driver function


if __name__ == "__main__":
    video = 0
    image = 1
    if video == 1:
        cap = cv2.VideoCapture(0)
        while True:
            # Capture frame-by-frame
            ret, img = cap.read()

            if not ret:
                print("Error: Unable to capture frame.")
                break
            else:
                img = Calibration.CalibrateCamera(img)
                # CourseDetection.giveMeCourseFramePoints(img)
                # displaying the image
                cv2.imshow('image', img)

                # setting mouse handler for the image
                # and calling the click_event() function
                cv2.setMouseCallback('image', click_event)

                # wait for a key to be pressed to exit
                cv2.waitKey(0)
                if hsv_values:
                    hsv_values = np.array(hsv_values)
                    min_hsv = hsv_values.min(axis=0)
                    max_hsv = hsv_values.max(axis=0)
                    print(f'Minimum HSV values: {min_hsv}')
                    print(f'Maximum HSV values: {max_hsv}')
                # close the window
                cv2.destroyAllWindows()
    elif image == 1:
        # img = Calibration.CalibrateCamera(img)
        # CourseDetection.giveMeCourseFramePoints(img)
        # displaying the image
        img = cv2.imread(image_path)
        cv2.imshow('image', img)

        # setting mouse handler for the image
        # and calling the click_event() function
        cv2.setMouseCallback('image', click_event)
        cv2.waitKey(0)

        if hsv_values:
            hsv_values = np.array(hsv_values)
            min_hsv = hsv_values.min(axis=0)
            max_hsv = hsv_values.max(axis=0)
            print(f'Minimum HSV values: {min_hsv}')
            print(f'Maximum HSV values: {max_hsv}')
        # wait for a key to be pressed to exit

        # close the window
        cv2.destroyAllWindows()
