import os
import cv2
import numpy as np

# Får den aktuelle mappe, hvor vores script ligger og den korrekte stig til billede filerne
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, '..', 'Data', 'Images', 'Robot_green2.jpg')
img = cv2.imread(image_path)

def detect_red(img):

    # definer de lysere farveområder hsv
    lower_red0 = np.array([0, 100, 100])
    upper_red0 = np.array([5, 255, 255])
    # definer de mørkere farveområder hsv
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([5, 255, 255])

    # Opret masker for lysere og mørkere rød
    mask1 = cv2.inRange(img, lower_red0, upper_red0)
    mask2 = cv2.inRange(img, lower_red1, upper_red1)
    mask = mask1 + mask2

    # Udfør morfologiske operationer
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Find konturer i masken og tegn dem på billedet
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 2)
    return img, mask

def define_path_within_frame(mask, img):
    # Find konturer i den røde maske for at definere banen
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:  # Tilpas områdetærskel efter behov
            x, y, w, h = cv2.boundingRect(contour)
            # Indsæt den faktiske bounding box for den blå bane her
            if x > some_value and x + w < some_other_value and y > some_value and y + h < some_other_value:
                cv2.drawContours(img, [contour], -1, (0, 0, 255), 2)
    return img

# Tjek om billedet bliver loadet rigtigt.
if img is None:
    print(f"Fejl: Kan ikke indlæse billedet fra '{image_path}'.")
else:
    # Konverter fra BGR til HSV farverummet
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Detect red areas in the image
    img_with_contours, mask = detect_red(img_hsv)
    # Define path within the frame
    path_img = define_path_within_frame(mask, img_with_contours)
    cv2.namedWindow('Course Detected', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Course Detected', 720, 1280)
    cv2.imshow('Course Detected', path_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()