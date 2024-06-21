import glob
import os

import cv2
import numpy as np

#Some taken from calibration script which comes from a youtuber and opencv
def something():
    chessboard_size = (9, 6)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    size_of_chessboard_squares_mm = 25
    objp = objp * size_of_chessboard_squares_mm

    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane

    script_dir = os.path.dirname(os.path.abspath(__file__))
    image_path1 = os.path.join(script_dir, '..', 'Images', 'Refiner2.png')
    image_path2 = os.path.join(script_dir, '..', 'Images', 'Refiner3.png')
    image_path3 = os.path.join(script_dir, '..', 'Images', 'Court4.png')
    image_path4 = os.path.join(script_dir, '..', 'Images', 'Refiner1.png')

    images = [image_path1, image_path2, image_path3, image_path4]

    for image in images:

        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(10000)

    cv2.destroyAllWindows()
    if len(objpoints) < 4:
          raise ValueError("Not enough points for homography calculation")

    objpoints = np.array(objpoints, dtype=np.float32).reshape(-1, 1, 3)
    imgpoints = np.array(imgpoints, dtype=np.float32).reshape(-1, 1, 2)
    print("Printing Object Points")
    print(objpoints)
    print("Printing Image Points")
    print(imgpoints)



if __name__ == "__main__":
    something()
