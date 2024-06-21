import cv2
import numpy as np

from Sandbox.TemporaryScripts.ImageAnalysisOnTestImage import ImageAnalysis
from src.Server.Components.RobotDetection import *


def ImageAnalysisOnVideo():
    # Open a connection to the camera (0 is usually the default camera)
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Unable to capture frame.")
            break
        else:
            ImageAnalysis(frame)
            # Break the loop if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()
