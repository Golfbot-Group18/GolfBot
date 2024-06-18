from Components.DetectionMethods import *


def DetectEgg(frame):
    min_canny_threshold = 100
    max_canny_threshold = 200
    min_ellipse_size = (30, 30)  # Minimum width and height of the ellipse
    max_ellipse_size = (100, 100)  # Maximum width and height of the ellipse
    eggs, _ = DetectEllipse(frame, min_ellipse_size, max_ellipse_size, min_canny_threshold, max_canny_threshold)
    return eggs
