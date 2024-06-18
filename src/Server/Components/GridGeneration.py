import numpy as np
import cv2
from shapely.geometry import Polygon
from scipy import ndimage

'''This function generates a grid from a binary image. The grid is a 2D list'''
def generate_grid(binary_image, interval=1):
    height, width = binary_image.shape
    grid = []
    for y in range(0, height, interval):
        row = []
        for x in range(0, width, interval):
            cell = binary_image[y:y + interval, x:x + interval]
            if np.any(cell == 255):  #Hvid er en forhindring.
                row.append(1)
            else:
                row.append(0)
        grid.append(row)
    return np.array(grid)

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



