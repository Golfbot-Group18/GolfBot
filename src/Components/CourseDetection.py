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

def define_inner_frame(mask, img):
    # Finder kanterne på objekter i en mask og tegner om dem
    x_min, x_max, y_min, y_max = 0, 1800, 0, 1200
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 5000:  # Område tærskel for at identificere signifikante objekter
            x, y, w, h = cv2.boundingRect(contour)
            if x > x_min and x + w < x_max and y > y_min and y + h < y_max:
                cv2.drawContours(img, [contour], 0, (0, 0, 255), 9)
    return img

def draw_coordinate_system(img, interval=100, color_x=(255, 0, 0), color_y=(0, 255, 0)):
    # Tegner koordinatsystem på billedet
    height, width = img.shape[:2]
    # X-akse
    for x in range(0, width, interval):
        cv2.line(img, (x, 0), (x, height), color_x, 1)
        cv2.putText(img, str(x), (x, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_x, 1)
    # Y-akse
    for y in range(0, height, interval):
        cv2.line(img, (0, y), (width, y), color_y, 1)
        cv2.putText(img, str(y), (width // 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_y, 1)

# Verifikation af at billede læses korrekt
if img is None:
    print(f"Fejl: Kan ikke indlæse billedet fra '{image_path}'.")
else:
    # Farvedetektion
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_lower = np.array([0, 50, 50])  # Lavere HSV grænse for rød
    red_upper = np.array([10, 255, 255])  # Øvre HSV grænse for rød
    img_with_red_contours, red_mask = detect_color(hsv_image, (red_lower, red_upper))

    # Anvend define_inner_frame og draw_coordinate_system
    path_img = define_inner_frame(red_mask, img)
    draw_coordinate_system(path_img, interval=100, color_x=(255, 0, 0), color_y=(0, 255, 0))

    # Vis det færdige resultat
    cv2.namedWindow('Course Detected', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Course Detected', 720, 1280)
    cv2.imshow('Course Detected', path_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
