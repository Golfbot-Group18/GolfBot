import os
import cv2
import numpy as np

# Get the current directory where your script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Provide the correct full path to the image file
image_path = os.path.join(script_dir, '..', 'Data', 'Images', 'test1.jpg')
# Load the image
img = cv2.imread(image_path)


