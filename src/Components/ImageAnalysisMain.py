import os
import cv2
import numpy as np

from src.Components.Course_David import Course_Detection
from src.Components.RobotDetection import RobotDetection

# Får den aktuelle mappe, hvor vores script ligger og den korrekte stig til billede filerne
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, '..', 'data', 'images', 'Robot_from.jpg')
img = cv2.imread(image_path)

Cx, Cy, Cw, Ch = Course_Detection.Detection(img)
#Rx, Ry, Rw, Rh = RobotDetection(img)

#For loop to create rectangles in one picture
cv2.rectangle(img, (Cx, Cy), (Cx + Cw, Cy + Ch), (0, 255, 0), 2)  # Green rectangle
# cv2.rectangle(img, (Rx, Ry), (Rx + Rw, Ry + Rh), (0, 255, 0), 2)


cv2.namedWindow('Image Analysis', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Image Analysis', 720, 1280)
cv2.imshow('Image Analysis', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
