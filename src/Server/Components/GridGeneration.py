import numpy as np
import cv2
from shapely.geometry import Polygon
from scipy import ndimage
from Components.EggDetection import DetectEgg
from Utils.px_conversion import realCoordinates

'''This function generates a grid from a binary image. The grid is a 2D list'''
def generate_grid(binary_image, frame, camera_height=165, interval=1):
    height, width = binary_image.shape
    grid = []
    gridCopy = []
    egg = DetectEgg(frame)
    
    #xpoints = np.array([])
    #ypoints = np.array([])


    # This now makes to lists; 
    # one for the grid of everything
    # a second which is a copy of it, which we will modify

    print("Generating the grid")
    #print("Finding the entire grid and the copy")
    for y in range(0, height, interval):
        row = []
        gridCopyRow = []

        for x in range(0, width, interval):
            cell = binary_image[y:y + interval, x:x + interval]

            if np.any(cell == 255):  #Hvid er en forhindring.
                row.append(1)
                gridCopyRow.append(1)
                #xpoints = np.append(xpoints,x)
                #ypoints = np.append(ypoints,y)
            else:
                row.append(0)
                gridCopyRow.append(0)
                               
        grid.append(row)
        gridCopy.append(gridCopyRow)
    
    # Now I have a copy that I will modify while scovering the 
    # original one

    #plt.plot(xpoints, ypoints, 'o')
    #plt.show()
    
    #xpoints = np.array([])
    #ypoints = np.array([])

    
    # Shifting top part of frame
    #print("Shifting top of frame")
    for x in range(0,len(grid[0])-1):
        for y in range (850,len(grid)-1):
            if(grid[y][x] == 1):
                shiftX, shiftY = realCoordinates(7.03,camera_height, (x,y))
                #xpoints = np.append(xpoints,shiftX)
                #ypoints = np.append(ypoints,shiftY)
                try:
                    gridCopy[y][x] = 0
                    gridCopy[int(shiftY)][int(shiftX)] = 1
                    
                except:
                    print("Unable to shift out of track")
    
    
    #plt.plot(xpoints, ypoints, 'o')
    #plt.show()
    
    #xpoints = np.array([])
    #ypoints = np.array([])
    

    
    # Shifting bottom part of frame
    #print("Shifting bottom of frame")
    for x in range(0,len(grid[0])-1):
        for y in range (0,150):
            if(grid[y][x] == 1):
                shiftX, shiftY = realCoordinates(7.03,camera_height, (x,y))
                #xpoints = np.append(xpoints,shiftX)
                #ypoints = np.append(ypoints,shiftY)
                try:
                    gridCopy[y][x] = 0
                    gridCopy[int(shiftY)][int(shiftX)] = 1
                    
                except:
                    print("Unable to shift out of frame")
    #plt.plot(xpoints, ypoints, 'o')
    #plt.show()
    

    #xpoints = np.array([])
    #ypoints = np.array([])

    
    # Shifting left part of frame
    #print("Shifting left of frame")
    for x in range(0,500):
        for y in range (0,len(grid)-1):
            if(grid[y][x] == 1):
                shiftX, shiftY = realCoordinates(7.03,camera_height, (x,y))
                #xpoints = np.append(xpoints,shiftX)
                #ypoints = np.append(ypoints,shiftY)
                try:
                    gridCopy[y][x] = 0
                    gridCopy[int(shiftY)][int(shiftX)] = 1
                    
                except:
                    print("Unable to shift out of frame")
    #plt.plot(xpoints, ypoints, 'o')
    #plt.show()
    

    #xpoints = np.array([])
    #ypoints = np.array([])

    
    # Shifting right part of frame
    #print("Shifting right of frame")
    for x in range(1500,len(grid[0])-1):
        for y in range (0,len(grid)-1):
            if(grid[y][x] == 1):
                shiftX, shiftY = realCoordinates(7.03,camera_height, (x,y))
                #xpoints = np.append(xpoints,shiftX)
                #ypoints = np.append(ypoints,shiftY)
                try:
                    gridCopy[y][x] = 0
                    gridCopy[int(shiftY)][int(shiftX)] = 1
                    
                except:
                    print("Unable to shift out of frame")

    #plt.plot(xpoints, ypoints, 'o')
    #plt.show()

    # Then all the coordinates of the frame should have been shifted



    #xpoints = np.array([])
    #ypoints = np.array([])


    # Not shifting the cross as it does some weird distortion
    # and also the cross height impact should be miniscule as it 
    # should be close to center and is only 3.05 cm in height

    '''
    # This is shifting the cross
    print("Shifting cross")
    for x in range(500,1500):
        for y in range(150, 800):
            if(grid[y][x] == 1):
                #Shift operation here
                # This is the height of the cross, camera height and the point.
                shiftX, shiftY = realCoordinates(3.05,165, (x,y))
                xpoints = np.append(xpoints,shiftX)
                ypoints = np.append(ypoints,shiftY)
                try: 
                    gridCopy[y][x] = 0
                    gridCopy[int(shiftY)][int(shiftX)] = 1
                    
                except:
                    print("Unable to shift out of frame")

    plt.plot(xpoints, ypoints, 'o')
    plt.show()

    xpoints = np.array([])
    ypoints = np.array([])
    '''



    # Need to insert the egg

    # Inserting the contour of the egg to both original grid and copy
    # And shifting it at the same time

    #print("Inserting the egg")
    for point in egg[0]:
        shiftX, shiftY = realCoordinates(6.68,camera_height, (point[0],point[1]))
        #xpoints = np.append(xpoints,shiftX)
        #ypoints = np.append(ypoints,shiftY)
        try: 
            grid[point[1]][point[0]] = 1
            gridCopy[int(shiftY)][int(shiftX)] = 1
        except:
            print("Unable to shift out of frame") 

        
    
    #plt.plot(xpoints, ypoints, 'o')
    #plt.show()

    '''

    xpoints = np.array([])
    ypoints = np.array([])

    print("Not Shifted")
    # plotting the original grid (with egg)
    for x in range(0,len(gridCopy[0])):
        for y in range(0,len(gridCopy)):
            if(gridCopy[y][x] == 1):
                xpoints = np.append(xpoints,x)
                ypoints = np.append(ypoints,y)

    plt.plot(xpoints, ypoints, 'o')
    plt.show()

    
    xpoints = np.array([])
    ypoints = np.array([])

    print("Final product")
    # plotting the shifted grid
    for x in range(0,len(gridCopy[0])):
        for y in range(0,len(gridCopy)):
            if(gridCopy[y][x] == 1):
                xpoints = np.append(xpoints,x)
                ypoints = np.append(ypoints,y)

    plt.plot(xpoints, ypoints, 'o')
    plt.show()

    '''
    
    print("Grid generated")
    return np.array(gridCopy)

'''This function takes a list of points refering to obstacles and creates a grid with 1s at the obstacle points and 0s elsewhere.'''
def create_obstacle_grid(obstacle_points, grid_shape):
    obstacle_grid = np.ones(grid_shape, dtype=np.int32)
    for point in obstacle_points:
        y, x = point
        obstacle_grid[y, x] = 0
    return obstacle_grid

'''This function creates a clearance grid from an obstacle grid. The clearance grid is a distance transform of the obstacle grid.'''
def create_clearance_grid(obstacle_grid):
    print("Unique values in obstacle grid:", np.unique(obstacle_grid))

    distance_transform = ndimage.distance_transform_edt(obstacle_grid)

    print("Max distance in distance transform:", np.max(distance_transform))

    max_distance = np.max(distance_transform)
    if max_distance == 0:
        normalized_clearance = np.zeros_like(distance_transform)
    else:
        normalized_clearance = (distance_transform / max_distance) * 100
    
    print("Clearance grid shape:", normalized_clearance.shape)
    print("Clearance grid max value:", np.max(normalized_clearance))
    print("Clearance grid min value:", np.min(normalized_clearance))

    return normalized_clearance, max_distance

'''This is just a helper function to analyze the clearance grid. It prints out some statistics about the clearance grid.'''
def analyze_clearance_grid(clearance_grid):
    max_clearance = np.max(clearance_grid)
    min_clearance = np.min(clearance_grid)
    mean_clearance = np.mean(clearance_grid)
    percentile_95 = np.percentile(clearance_grid, 95)
    
    print(f"Max clearance: {max_clearance}")
    print(f"Min clearance: {min_clearance}")
    print(f"Mean clearance: {mean_clearance}")
    print(f"95th percentile clearance: {percentile_95}")
    
    return max_clearance, min_clearance, mean_clearance, percentile_95

'''This function is currently not in use, but it's purpose is to remove the x from the grid. It was made when we tried to find the inner frame of course obstacles'''
def remove_x_from_grid(grid, x_range, y_range, interval):
    x_start, x_end = x_range
    y_start, y_end = y_range
    
    x_start_idx = x_start // interval
    x_end_idx = (x_end - 1 + interval) // interval  
    y_start_idx = y_start // interval
    y_end_idx = (y_end - 1 + interval) // interval 
    
    print(f"Removing X from grid at indices: x({x_start_idx}:{x_end_idx}), y({y_start_idx}:{y_end_idx})")
    
    grid[y_start_idx:y_end_idx, x_start_idx:x_end_idx] = 2
    return grid

'''This function finds the coordinates of the obstacles in the grid. It returns a list of coordinates'''
def find_obstacle_coords(grid):
    coords = np.where(grid == 1)
    if len(coords[0]) == 0 or len(coords[1]) == 0:
        return None

    return list(zip(coords[0], coords[1]))

'''A helper function to visualize the clearance grid. It returns a colored image of the clearance grid.'''
def visualize_clearance_grid(clearance_grid, interval=10):
    clearance_grid_resized = cv2.resize(clearance_grid, (clearance_grid.shape[1] // interval, clearance_grid.shape[0] // interval))
    clearance_grid_normalized = cv2.normalize(clearance_grid_resized, None, 0, 255, cv2.NORM_MINMAX)
    clearance_grid_colored = cv2.applyColorMap(clearance_grid_normalized.astype(np.uint8), cv2.COLORMAP_JET)
    return clearance_grid_colored

'''A helper function to visualize the grid created from the binary image. It returns a black and white image of the grid.'''
def visualize_grid(grid, interval=10):
    height, width = grid.shape
    vis_img = np.ones((height * interval, width * interval, 3), np.uint8) * 255

    for y in range(height):
        for x in range(width):
            if grid[y, x] == 1:
                vis_img[y * interval:(y + 1) * interval, x * interval:(x + 1) * interval] = (0, 0, 0)

    return vis_img

'''A helper function to visualize the grid with a path. It returns a black and white image of the grid with a red path drawn on top.'''
def visualize_grid_with_path(grid, interval=10, path=[]):
    vis_img = visualize_grid(grid, interval)

    for (y, x) in path:
        cv2.circle(vis_img, (x * interval + interval // 2, y * interval + interval // 2), interval // 4, (0, 0, 255), -1)

    return vis_img

'''A function to draw the egg as an obstacle on the grid. It returns the grid with the egg drawn as an obstacle, with a bufferzone'''
def draw_egg_as_obstacle(egg_points, obstacle_grid, buffersize=10):
    height, width = obstacle_grid.shape
    for point in egg_points:
        x, y = point
        for i in range(max(0, y - buffersize), min(height, y + buffersize + 1)):
            for j in range(max(0, x - buffersize), min(width, x + buffersize + 1)):
                obstacle_grid[i, j] = 1
    return obstacle_grid


def add_contour_to_obstacle_grid(grid, contour):
    contour_points = np.array(contour).reshape((-1, 1, 2))
    mask = np.zeros(grid.shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour_points], -1, 255, thickness=cv2.FILLED)
    kernel = np.ones((9,9), np.uint8)
    dilated_mask = cv2.dilate(mask, kernel, iterations=1)

    grid[dilated_mask > 0] = 0

    return grid

def remove_contour_to_obstacle_grid(grid, contour):
    contour_points = np.array(contour).reshape((-1, 1, 2))
    mask = np.zeros(grid.shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour_points], -1, 255, thickness=cv2.FILLED)
    kernel = np.ones((9,9), np.uint8)
    dilated_mask = cv2.dilate(mask, kernel, iterations=1)

    grid[dilated_mask > 0] = 1

    return grid
