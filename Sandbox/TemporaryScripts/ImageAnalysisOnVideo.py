import cv2
import numpy as np

from Sandbox.TemporaryScripts.ImageAnalysisOnTestImage import ImageAnalysis
from src.Server.Components.RobotDetection import *


def ImageAnalysisOnVideo():
    # Open a connection to the camera (0 is usually the default camera)
    cap = cv2.VideoCapture(0)
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    #cap.set(cv2.CAP_PROP_FPS, 60)
    #print(cap.get(cv2.CAP_PROP_FPS))
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()


        if not ret:
            print("Error: Unable to capture frame.")
            break
        else:
            ImageAnalysis(frame)
            #cv2.imshow('Video', frame)
            # Break the loop if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()
