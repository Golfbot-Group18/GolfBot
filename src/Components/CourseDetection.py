import os
import cv2
import numpy as np

# Får den aktuelle mappe, hvor vores script ligger og den korrekte stig til billede filerne
script_dir = os.path.dirname(__file__))
image_path = os.path.join(script_dir, '..', 'data', 'images', 'Robot_green-jpg')
img = cv2.imread(image_path)

# Konverter fra BGR til HSV farverummet
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
def detect_red(img_hsv):

    # definer de lysere farveområder hsv
    lower_red0 = np.array([0, 100, 100])
    upper_red0 = np.array([5, 255, 255])
    # definer de mørkere farveområder hsv
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([5, 255, 255])

