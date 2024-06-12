import os
import cv2
import numpy as np
print(os.getcwd)
from src.Server.Components.MainImageAnalysis import giveMeFrames
from src.Server.Components.CourseDetection import generate_grid, visualize_grid, giveMeBinaryBitch, visualize_grid_with_path
from src.Server.Pathfinding.Pathfinding import a_star_search
import cv2

# Need to get image in first

# Get the current directory where your script is located
#script_dir = os.path.dirname(os.path.abspath(__file__))

#image_path = os.path.join(script_dir, '..', 'Outdated', 'Images', 'Robot_green2.jpg')
# Load the image
img = cv2.imread("src\Server\DataProcessing\Robot_green2.jpg")
SOMETHING = 0

# Display the image
cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Image', 1080, 1920)
cv2.imshow("Image", img)




binary_course = giveMeBinaryBitch(img)
# Wait for the user to press a key
cv2.waitKey(0)
 
# Close all windows
cv2.destroyAllWindows()