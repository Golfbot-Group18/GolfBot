import os
import cv2
import numpy as np
import sys
import math
import matplotlib.pyplot as plt
from Components.MainImageAnalysis import giveMeFrames
from Components.CourseDetection import generate_grid, visualize_grid, giveMeBinaryBitch, visualize_grid_with_path
from Pathfinding.Pathfinding import a_star_search

# Load the image
img = cv2.imread("src\Server\Robot_green.jpg")

# Files available

# src\Server\straight_cross.jpg
# src\Server\cross_cross.jpg
# src\Server\Robot_green2.jpg
# src\Server\Empty_course.jpeg
# src\Server\Robot_from.jpg
# src\Server\Robot_green.jpg


# Setting up to display the image
cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Image', 1080, 1920)

# Converting the image to an binary image
binary_course = giveMeBinaryBitch(img)


# trial and error to narrow done the cross in the middle. 
# From the image Robot_green2 I think the interval should be
# (300..600, 700..1100)

# REVISED 
# With the current way the interval is set up
# The interval should be fine as long as one uses
# a fairly resanoable interval. 

# It directs from extremities, so if noise is in the 
# extremities then the program will fault



def scaleBitchPlease(binary_coure:np.ndarray)->float:


    # These two are only to display the points in a grid
    xpoints = np.array([])
    ypoints = np.array([])

    # These are local Minima's from the
    # most northly point 
    NorthLocalMinimaLeft = (0,0)
    NorthLocalMinimaRight = (0,0)


    # These are local Minima's from the
    # most sourtherly point 
    SouthLocalMinimaLeft = (3000,3000)
    SouthLocalMinimaRight = (3000,3000)


    # These are local Minima's from the
    # most westerly point 
    WestLocalMinimaBottom= (0,0)
    WestLocalMinimaTop = (0,0)


    # These are local Minima's from the
    # most easterly point 
    EastLocalMinimaBottom= (3000,3000)
    EastLocalMinimaTop = (3000,3000)


    # These are the initalization of the
    # extremities 
    north = (1100,600)
    south = (300, 600)
    west = (600, 1100)
    east = (600, 700)



    #This first loop is to find the extremities

    # origo is in top left
    # y range
    for i in range (300,600):
        # x range
        for j in range (700,1100):
            #if it is registered as red
            if binary_course[i][j] == 255: 
                #Most northern
                if i<= north[0]:
                    north=(i,j)
                #Most southern
                if i>= south[0]:
                    south=(i,j)
                #Most western
                if j<= west[1]:
                    west = (i,j)
                #Most eastern
                if j>= east[1]:
                    east = (i,j)
            
            

    # Showing the findings in prompt
    print("The extremeties")
    print(f'north:{north}')
    print(f'south:{south}')
    print(f'west:{west}')
    print(f'east:{east}\n')

    # Saving them for later illustration
    xpoints = np.append(xpoints,[north[1],south[1],west[1],east[1]])
    ypoints = np.append(ypoints,[-north[0],-south[0],-west[0],-east[0]])


    # Displaying the range we'll follow from the most northerly point
    # The range is subject to change as it might be unecessarely large
    #print(f'range: {north[1]-500} .. {north[1]+500}')


    # I can effectively map the top of the cross with the following script




    ######################################################
        
    # Need to find North A and B



    #This goes from left to right
    # true north
    prevCounter = 0
    for j in range (north[1], north[1]+400):
        # I need to change i to go down to nearest white on that line
        # j starts from true north moves a pixel to the right
        i = north[0]

        #Counter to check how far down one needs to 
        # travel to find the nearest hit/point
        counter = 0
        
        #This while loop identifies the i value 
        # to the nearest filled value from north
        while(binary_course[i][j] != 255):
            i += 1
            counter += 1
            
        #If the value is within 5 pixels of the last value
        # found then it will be added.
        # otherwise it is too far away and it is a "jump"
        if(counter < prevCounter+5): 
            xpoints = np.append(xpoints,j)
            ypoints = np.append(ypoints,-i)
            prevCounter = counter
            #If it is the lowest point found
            if i >= NorthLocalMinimaRight[0]:
                NorthLocalMinimaRight = (i,j)
        else: 
            break
        
    # Showing the findings in prompt
    print("The minimas for north:")
    print(f'NorthLocalMinimaRight {NorthLocalMinimaRight}')



    #This is to look from right to left from 
    # true north
    prevCounter = 0
    for j in range (north[1], north[1]-400, -1):
        # I need to change j to go down to nearest white on that line
        i = north[0]

        #Counter to check how far down one needs to 
        # travel to find the nearest hit/point
        counter = 0
        
        #This while loop identifies the i value 
        # to the nearest filled value from north
        while(binary_course[i][j] != 255):
            i += 1
            counter += 1
            
        #If the value is within 5 pixels of the last value
        # found then it will be added.
        # otherwise it is too far away and it is a "jump"
        if(counter < prevCounter+5): 
            xpoints = np.append(xpoints,j)
            ypoints = np.append(ypoints,-i)
            prevCounter = counter
            #If it is the lowest point found
            if i >= NorthLocalMinimaLeft[0]:
                NorthLocalMinimaLeft = (i,j)
        else:
            break

    # Showing the findings in prompt
    print(f'NorthLocalMinimaLeft: {NorthLocalMinimaLeft}\n')




    ######################################################
        
    # Need to find south A and B



    #This goes from left to right
    # true south
    prevCounter = 0
    for j in range (south[1], south[1]+400):
        # I need to change i to go up to nearest white on that line
        # j starts from true south moves a pixel to the right
        i = south[0]

        #Counter to check how far up one needs to 
        # travel to find the nearest hit/point
        counter = 0
        
        #This while loop identifies the i value 
        # to the nearest filled value from south
        while(binary_course[i][j] != 255):
            i -= 1
            counter += 1
            
        #If the value is within 5 pixels of the last value
        # found then it will be added.
        # otherwise it is too far away and it is a "jump"
        if(counter < prevCounter+5): 
            xpoints = np.append(xpoints,j)
            ypoints = np.append(ypoints,-i)
            prevCounter = counter
            #If it is the lowest point found
            if i <= SouthLocalMinimaRight[0]:
                SouthLocalMinimaRight = (i,j)
        else: 
            break
        
    # Showing the findings in prompt
    print("\nThe minimas for south:")
    print(f'SouthLocalMinimaRight {SouthLocalMinimaRight}')



    #This is to look from right to left from 
    # true south
    prevCounter = 0
    for j in range (south[1], south[1]-400, -1):
        # I need to change j to go up to nearest white on that line
        i = south[0]

        #Counter to check how far up one needs to 
        # travel to find the nearest hit/point
        counter = 0
        
        #This while loop identifies the i value 
        # to the nearest filled value from north
        while(binary_course[i][j] != 255):
            i -= 1
            counter += 1
            
        #If the value is within 5 pixels of the last value
        # found then it will be added.
        # otherwise it is too far away and it is a "jump"
        if(counter < prevCounter+5): 
            xpoints = np.append(xpoints,j)
            ypoints = np.append(ypoints,-i)
            prevCounter = counter
            #If it is the lowest point found
            if i <= SouthLocalMinimaLeft[0]:
                SouthLocalMinimaLeft = (i,j)
        else:
            break

    # Showing the findings in prompt
    print(f'SouthLocalMinimaLeft: {SouthLocalMinimaLeft}\n')




    ###########################################

    # Need to find A and B for west


    #This goes from up to down
    # true West
    prevCounter = 0
    for i in range (west[0], west[0]+300):
        # I need to change j to go right to nearest white on that line
        # i starts from true west moves a pixel down
        j = west[1]

        #Counter to check how far up one needs to 
        # travel to find the nearest hit/point
        counter = 0
        
        #This while loop identifies the i value 
        # to the nearest filled value from west
        while(binary_course[i][j] != 255):
            j += 1
            counter += 1
            
        #If the value is within 5 pixels of the last value
        # found then it will be added.
        # otherwise it is too far away and it is a "jump"
        if(counter < prevCounter+5): 
            xpoints = np.append(xpoints,j)
            ypoints = np.append(ypoints,-i)
            prevCounter = counter
            #If it is the lowest point found
            if j >= WestLocalMinimaBottom[1]:
                WestLocalMinimaBottom= (i,j)
        else: 
            break
        
    # Showing the findings in prompt
    print("\nThe minimas for West:")
    print(f'WestLocalMinimaBottom {WestLocalMinimaBottom}')



    #This is to look from bottom to top from 
    # true West
    prevCounter = 0
    for i in range (west[0], west[0]-300, -1):
        # I need to change i to go to nearest white on that line
        j = west[1]

        #Counter to check how far up one needs to 
        # travel to find the nearest hit/point
        counter = 0
        
        #This while loop identifies the i value 
        # to the nearest filled value from north
        while(binary_course[i][j] != 255):
            j += 1
            counter += 1
            
        #If the value is within 5 pixels of the last value
        # found then it will be added.
        # otherwise it is too far away and it is a "jump"
        if(counter < prevCounter+5): 
            xpoints = np.append(xpoints,j)
            ypoints = np.append(ypoints,-i)
            prevCounter = counter
            #If it is the lowest point found
            if j >= WestLocalMinimaTop[1]:
                WestLocalMinimaTop = (i,j)
        else:
            break

    # Showing the findings in prompt
    print(f'WestLocalMinimaTop: {WestLocalMinimaTop}\n')




    ###########################################

    # Need to find A and B for East


    #This goes from up to down
    # true East
    prevCounter = 0
    for i in range (east[0], east[0]+300):
        # I need to change j to go left to nearest white on that line
        # i starts from true west moves a pixel down
        j = east[1]

        #Counter to check how far up one needs to 
        # travel to find the nearest hit/point
        counter = 0
        
        #This while loop identifies the i value 
        # to the nearest filled value from west
        while(binary_course[i][j] != 255):
            j -= 1
            counter += 1
            
        #If the value is within 5 pixels of the last value
        # found then it will be added.
        # otherwise it is too far away and it is a "jump"
        if(counter < prevCounter+5): 
            xpoints = np.append(xpoints,j)
            ypoints = np.append(ypoints,-i)
            prevCounter = counter
            #If it is the lowest point found
            if j <= EastLocalMinimaBottom[1]:
                EastLocalMinimaBottom= (i,j)
        else: 
            break
        
    # Showing the findings in prompt
    print("\nThe minimas for East:")
    print(f'EastLocalMinimaBottom {EastLocalMinimaBottom}')



    #This is to look from bottom to top from 
    # true East
    prevCounter = 0
    for i in range (east[0], east[0]-300, -1):
        # I need to change i to go to nearest white on that line
        j = east[1]

        #Counter to check how far up one needs to 
        # travel to find the nearest hit/point
        counter = 0
        
        #This while loop identifies the i value 
        # to the nearest filled value from north
        while(binary_course[i][j] != 255):
            j -= 1
            counter += 1
            
        #If the value is within 5 pixels of the last value
        # found then it will be added.
        # otherwise it is too far away and it is a "jump"
        if(counter < prevCounter+5): 
            xpoints = np.append(xpoints,j)
            ypoints = np.append(ypoints,-i)
            prevCounter = counter
            #If it is the lowest point found
            if j <= EastLocalMinimaTop[1]:
                EastLocalMinimaTop = (i,j)
        else:
            break

    # Showing the findings in prompt
    print(f'EastLocalMinimaTop: {EastLocalMinimaTop}\n')







    # If the difference between the to minima's x coordinates 
    # are larger then the difference between the to minima's
    # y coordinates than the cross is askew. 


    pixelScale = -1

    # This is the real value of A in cm
    realA = 2

    # This is the real value of B in cm
    realB = 5



    # The cross is defined as askew if the difference between local minimas is 
    # larger than 10 pixels

    xNDifference = abs(NorthLocalMinimaRight[1]-NorthLocalMinimaLeft[1])
    yNDifference = abs(NorthLocalMinimaLeft[0]-NorthLocalMinimaRight[0])

    xSDifference = abs(SouthLocalMinimaLeft[1]-SouthLocalMinimaRight[1])
    ySDifference = abs(SouthLocalMinimaRight[0]-SouthLocalMinimaLeft[0])

    xWDifference = abs(WestLocalMinimaBottom[1]-WestLocalMinimaTop[1])
    yWDifference = abs(WestLocalMinimaBottom[0]-WestLocalMinimaTop[0])

    yEDifference = abs(EastLocalMinimaBottom[0]-EastLocalMinimaTop[0])
    xEDifference = abs(EastLocalMinimaBottom[1]-EastLocalMinimaTop[1])

    print(f'yDifference: N = {yNDifference}, S = {ySDifference}')
    print(f'xDifference: W = {xWDifference}, E = {xEDifference}')

    aAverage = -1
    bAverage = -1

    #If the cross is askew
    if((10 < yNDifference or 10 < ySDifference) and (10 < xWDifference or 10 < xEDifference)):
        
        print("Cross is askew")

        # Side A and side B is then identified
        # See conversion_illustration.jpg for reference

        
        # If side A is to the right of true north 
        if(NorthLocalMinimaLeft[0]>NorthLocalMinimaRight[0]):
            sum = (NorthLocalMinimaRight[1]-north[1])**2+(NorthLocalMinimaRight[0]-north[0])**2

            aPixelN = math.sqrt(sum)
            
            sum = (north[1]-NorthLocalMinimaLeft[1])**2 + (north[0]-NorthLocalMinimaLeft[0])**2

            bPixelN = math.sqrt(sum)


        # If side A is to the left of true north
        else: 
            sum = (NorthLocalMinimaRight[1]-north[1])**2+(NorthLocalMinimaRight[0]-north[0])**2

            bPixelN = math.sqrt(sum)
            
            sum = (north[1]-NorthLocalMinimaLeft[1])**2 + (north[0]-NorthLocalMinimaLeft[0])**2

            aPixelN = math.sqrt(sum)



        # If side A is to the right of true south 
        if(SouthLocalMinimaLeft[0]<SouthLocalMinimaRight[0]):
            sum = (SouthLocalMinimaRight[1]-south[1])**2+(SouthLocalMinimaRight[0]-south[0])**2

            aPixelS = math.sqrt(sum)
            
            sum = (south[1]-SouthLocalMinimaLeft[1])**2 + (south[0]-SouthLocalMinimaLeft[0])**2

            bPixelS = math.sqrt(sum)


        # If side A is to the left of true south
        else: 
            sum = (SouthLocalMinimaRight[1]-south[1])**2+(SouthLocalMinimaRight[0]-south[0])**2

            bPixelS = math.sqrt(sum)
            
            sum = (south[1]-SouthLocalMinimaLeft[1])**2 + (south[0]-SouthLocalMinimaLeft[0])**2

            aPixelS = math.sqrt(sum)




        # If side A is to the above of true west 
        if(WestLocalMinimaTop[1]<WestLocalMinimaBottom[1]):
            sum = (WestLocalMinimaTop[1]-west[1])**2+(WestLocalMinimaTop[0]-west[0])**2

            aPixelW = math.sqrt(sum)
            
            sum = (west[1]-WestLocalMinimaBottom[1])**2 + (west[0]-WestLocalMinimaBottom[0])**2

            bPixelW = math.sqrt(sum)


        # If side A is to the bellow of true west
        else: 
            sum = (WestLocalMinimaBottom[1]-west[1])**2+(WestLocalMinimaBottom[0]-west[0])**2

            aPixelW = math.sqrt(sum)
            
            sum = (west[1]-WestLocalMinimaTop[1])**2 + (west[0]-WestLocalMinimaTop[0])**2

            bPixelW = math.sqrt(sum)
        




        # If side A is to the above of true east 
        if(EastLocalMinimaTop[1]>EastLocalMinimaBottom[1]):
            sum = (EastLocalMinimaTop[1]-east[1])**2+(EastLocalMinimaTop[0]-east[0])**2

            aPixelE = math.sqrt(sum)
            
            sum = (east[1]-EastLocalMinimaBottom[1])**2 + (east[0]-EastLocalMinimaBottom[0])**2

            bPixelE = math.sqrt(sum)


        # If side A is to the bellow of true east
        else: 
            sum = (EastLocalMinimaBottom[1]-east[1])**2+(EastLocalMinimaBottom[0]-east[0])**2

            aPixelE = math.sqrt(sum)
            
            sum = (east[1]-EastLocalMinimaTop[1])**2 + (east[0]-EastLocalMinimaTop[0])**2

            bPixelE = math.sqrt(sum)

        aList = [aPixelN,aPixelS,aPixelW,aPixelE]
        bList = [bPixelN,bPixelS,bPixelW,bPixelE]

        aAverage = math.fsum(aList)/len(aList)

        # removing outliers to get a better normal distribution
        # i.e. better average
        counter = 0
        while(counter < len(aList)):
            if(abs(aList[counter]-aAverage)>3):
                aList.pop(counter)
                aAverage = math.fsum(aList)/len(aList)
                counter = 0
            else:
                counter += 1
            


        bAverage = math.fsum(bList)/len(bList)

        counter = 0
        while(counter < len(bList)):
            if(abs(bList[counter]-bAverage)>3):
                bList.pop(counter)
                bAverage = math.fsum(bList)/len(bList)
                counter = 0
            else:
                counter += 1


        print(f'aPixel: n={aPixelN}, s={aPixelS}, w={aPixelW}, e={aPixelE}')
        print(f'bPixel: n={bPixelN}, s={bPixelS}, w={bPixelW}, e={bPixelE}')

        print(f'aAverage: {aAverage}')
        print(f'bAverage: {bAverage}')



    # In case the cross is center, i.e. not askew
    else:
        print("Cross is center, i.e. NOT askew")

        #Side A is then found as B is not able to be found from local maxima
        # difference between local maxima's is the length of A

        aPixelN = NorthLocalMinimaRight[1]-NorthLocalMinimaLeft[1]
        aPixelS = SouthLocalMinimaRight[1]-SouthLocalMinimaLeft[1]
        aPixelW = WestLocalMinimaBottom[0]-WestLocalMinimaTop[0]
        aPixelE = EastLocalMinimaBottom[0]-EastLocalMinimaTop[0]


        aList = [aPixelN,aPixelS,aPixelW,aPixelE]
        aAverage = math.fsum(aList)/len(aList)

        # removing outliers to get a better normal distribution
        # i.e. better average
        counter = 0
        while(counter < len(aList)):
            if(abs(aList[counter]-aAverage)>3):
                aList.pop(counter)
                aAverage = math.fsum(aList)/len(aList)
                counter = 0
            else:
                counter += 1

        


        print(f'aPixelN: {aPixelN}')
        print(f'aPixelS: {aPixelS}')
        print(f'aPixelW: {aPixelW}')
        print(f'aPixelE: {aPixelE}')

        print(f'aAverage: {aAverage}')




    cv2.imshow("Image", binary_course)


    plt.plot(xpoints, ypoints, 'o')
    plt.show()


    # Wait for the user to press a key
    cv2.waitKey(0)
    
    # Close all windows
    cv2.destroyAllWindows()

    return aAverage, bAverage


scaleBitchPlease(binary_course)