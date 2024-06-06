import os
import cv2
import numpy as np

# Får den aktuelle mappe, hvor vores script ligger og den korrekte sti til billede filerne
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, '..', 'Data', 'Images', 'Robot_green2.jpg')
img = cv2.imread(image_path)

def detect_color(img, color_range):
    mask = cv2.inRange(img, color_range[0], color_range[1])
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 2)
    return img, mask

def convert_to_binary(img, threshold_value=128):
    # Konverter billedet til gråskala
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Anvend binær tærskelværdi
    _, binary_image = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY)
    return binary_image

if img is None:
    print(f"Fejl: Kan ikke indlæse billedet fra '{image_path}'.")
else:
    # Konverter fra BGR til HSV farverummet
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Farveområder i HSV
    red_ranges = (
        np.array([0, 100, 100]), np.array([5, 255, 255]),
        np.array([170, 100, 100]), np.array([180, 255, 255])
    )
    yellow_range = (np.array([20, 100, 100]), np.array([30, 255, 255]))

    # Detekter rød
    _, red_mask = detect_color(hsv, red_ranges[:2])
    _, red_mask2 = detect_color(hsv, red_ranges[2:])
    total_red_mask = red_mask + red_mask2

    # Detekter gul
    hsv, yellow_mask = detect_color(hsv, yellow_range)

    # Konverter HSV-billede til binært billede
    binary_image = convert_to_binary(hsv)

    # Vis det bearbejdet billede og det binære billede
    cv2.namedWindow('Course Detected', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Course Detected', 720, 1280)
    cv2.imshow('Course Detected', hsv)
    cv2.imshow('Binary Image', binary_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
