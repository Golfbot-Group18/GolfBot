import numpy as np
import cv2 as cv
import sys
import os
from ultralytics import YOLO

capture = cv.VideoCapture(0)
if not capture.isOpened():
    print("Error opening video stream or file")
    exit()

while True:
    # Capturing frame-by-frame
    ret, frame = capture.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Cannot recieve frame (stream end?). Exiting... ")
        break

    # Our operations on the frame come here
    model = YOLO('yolov8n.pt')
    results = model(source=0, show=True, conf=0.4, save=True)

    frame_ = results[0].plot()

    # Display the resulting frame
    cv.imshow('frame', frame)
    # If 'q' button is pressed then quit
    if cv.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
capture.release()
cv.destroyAllWindows()
