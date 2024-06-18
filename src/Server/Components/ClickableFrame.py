# THIS CODE IS TAKEN FROM GEEKS FOR GEEKS AND IS INTENDED AS TESTING CODE TO FIND POINTS
# ON THE SCREEEN WHEN CLICKS

# importing the module
import cv2
import src.Server.Camera.Calibration as Calibration
import src.Server.Components.CourseDetection as CourseDetection


# function to display the coordinates of
# of the points clicked on the image
def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)

        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(x) + ',' +
                    str(y), (x, y), font,
                    1, (255, 0, 0), 2)
        cv2.imshow('image', img)

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)

        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        cv2.putText(img, str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x, y), font, 1,
                    (255, 255, 0), 2)
        cv2.imshow('image', img)

    # driver function


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while True:
        # Capture frame-by-frame
        ret, img = cap.read()

        if not ret:
            print("Error: Unable to capture frame.")
            break
        else:
            img = Calibration.CalibrateCamera(img)
            # CourseDetection.giveMeCourseFramePoints(img)
            # displaying the image
            cv2.imshow('image', img)

            # setting mouse handler for the image
            # and calling the click_event() function
            cv2.setMouseCallback('image', click_event)

            # wait for a key to be pressed to exit
            cv2.waitKey(0)

            # close the window
            cv2.destroyAllWindows()
