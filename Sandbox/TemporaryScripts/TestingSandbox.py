import os
import cv2
import numpy as np

from Sandbox.TemporaryScripts.ImageAnalysisOnTestImage import ImageAnalysisOnTestImage
from Sandbox.TemporaryScripts.ImageAnalysisOnVideo import ImageAnalysisOnVideo
from src.Server.Components.CourseDetection import *

script_dir = os.path.dirname(os.path.abspath(__file__))

# Provide the correct full path to the image file
image_path = os.path.join(script_dir, '..', 'Images', 'RobotTriangle8.png')

if __name__ == "__main__":

    video = 0
    image = 1

    if video == 1:
        ImageAnalysisOnVideo()
    elif image == 1:
        ImageAnalysisOnTestImage(image_path)
    else:
        raise ValueError("Invalid choice")
