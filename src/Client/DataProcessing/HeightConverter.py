import math 

# This is to account for the height of the robot when the camera reads the position.
# For this I need the following arguments 

#   - Height of the camera above the ground (and the camera needs to be level)
#   - Coordinats directly beneath the robot. 
#   - Coordinats that the robot thinks it is at (as told by the camera)

# all measurements will be in cm



# To account for the height displacement of the robot, all in cm
def realCoordinates (robotHeight: float, cameraHeight: float, robotCoordinates):
    
    #I'm at the moment assuming that directly beneath the camera is the center of the 
    # grid. This should always be the case if the camera is level. 
    cameraCoordinates = (960,540)



    # d is the distance from the ground point of the robot to the ground point of the camera 
    d1 = math.sqrt((robotCoordinates[0]-cameraCoordinates[0])**2-(cameraCoordinates[1]-robotCoordinates[1])**2)

    # Angle C is the angle from the ground where it thinks the robot is at
    # to the Camera 
    C = math.atan(cameraHeight/d1)

    # R is the real angle from the robot to the camera
    # which should be the same as the angle C
    R = C

    # d2 is the real distance from the ground point of the camera to the robot
    d2 = (cameraHeight-robotHeight)/math.tan(R)

    # I then find the scale to use on the coordinates of the robot
    scale = d2/d1

    # I then find the real coordinates of the robot
    x = robotCoordinates[0]*scale
    y = robotCoordinates[1]*scale

    return x,y



    
    
    '''
    b1 = math.sqrt((cameraCoordinates[0]-robotCoordinates[0])**2+(cameraCoordinates[1]-robotCoordinates[1])**2)

    A = math.atan(cameraHeight/b1)

    b2 = (cameraHeight-robotHeight)/(math.tan(A))

    C = math.atan((cameraCoordinates[0]-robotCoordinates[0])/(cameraCoordinates[1]-robotCoordinates[1]))

    x = math.sin(C)*b2

    y = math.cos(C)*b2

    print(f'x: {x}, y: {y}, length: {b2}')

    return x, y, b2
    '''





robotHeight = 15.19

# camera height needs to more accuratly measured
cameraHeight = 161

# it needs (x,y)

robotCoordinates = (96,54)

test = (400,152)
x, y = realCoordinates(robotHeight, cameraHeight, test)

print("First x and y")
print(f'x: {test[0]}')
print(f'y: {test[1]}')


print("\nModified")
print(f'x: {x}')
print(f'y: {y}')
