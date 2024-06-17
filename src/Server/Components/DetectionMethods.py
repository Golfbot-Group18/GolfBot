import numpy as np
import cv2


def DetectEllipse(frame, min_ellipse_size, max_ellipse_size, min_canny_threshold, max_canny_threshold):
    grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(grey_image, 230, 255, cv2.THRESH_BINARY)
    inverted_image = cv2.bitwise_not(binary_image)

    # Gaussian blur
    img_size = (9, 9)
    gaussian_image = cv2.GaussianBlur(inverted_image, img_size, 4)

    # Canny edge detector
    canny_image = cv2.Canny(gaussian_image, min_canny_threshold, max_canny_threshold)

    # Find contours
    contours, _ = cv2.findContours(canny_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Fit ellipses to the contours
    ellipse_contours = []
    for contour in contours:
        if len(contour) >= 5:  # FitEllipse needs at least 5 points
            ellipse = cv2.fitEllipse(contour)
            width, height = ellipse[1]
            if min_ellipse_size[0] <= width <= max_ellipse_size[0] and min_ellipse_size[1] <= height <= \
                    max_ellipse_size[1]:
                ellipse_contour = cv2.ellipse2Poly((int(ellipse[0][0]), int(ellipse[0][1])),
                                                   (int(width / 2), int(height / 2)),
                                                   int(ellipse[2]), 0, 360, 10)
                ellipse_contours.append(ellipse_contour)
    return ellipse_contours


def DetectColor(frame, lower, upper):
    # Konverter fra BGR til HSV farverummet
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # img_hsv[..., 1] = img_hsv[..., 1]*1.1

    # Mask the image to find all green areas
    mask = cv2.inRange(img_hsv, lower, upper)

    # Returns a list of contours, and the second value is a hierarchy (which we don’t need in this case).
    # Hence, the underscore
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours by area (you can adjust the threshold)
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 3000]

    if filtered_contours:
        # Get the largest contour
        largest_contour = max(filtered_contours, key=cv2.contourArea)

        # x, y, w, h = cv2.boundingRect(largest_contour)
        return largest_contour
    else:
        # print("No green area found in the image.")
        return None
