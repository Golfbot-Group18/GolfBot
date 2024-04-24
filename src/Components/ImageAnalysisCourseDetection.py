import os
import cv2
import numpy as np

# Provide the correct full path to the image file
image_path = os.path.join(os.getcwd(), 'Images', 'test1.jpg')
# Load the image
img = cv2.imread(image_path)

# Check if the image is loaded successfully
if img is None:
    print(f"Error: Unable to load the image from '{image_path}'.")
else:
    # Convert to HSV color space
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define red color ranges
    lower_red = np.array([0, 20, 50])
    upper_red = np.array([5, 255, 255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    lower_red = np.array([170, 20, 20])
    upper_red = np.array([180, 255, 255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    # Combine masks
    mask = mask0 + mask1

    kernel = np.ones((5, 5), "uint8")

    mask = cv2.dilate(mask, kernel)
    res_red = cv2.bitwise_and(img, img,
                              mask=mask)

    contours, hierarchy = cv2.findContours(mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(img, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)

            cv2.putText(imageFrame, "Red Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))

    cv2.imshow('Course', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
