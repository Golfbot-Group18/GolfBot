import numpy as np
import cv2
from shapely.geometry import Polygon
from scipy import ndimage

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

def create_obstacle_grid(obstacle_points, grid_shape):
    obstacle_grid = np.ones(grid_shape, dtype=np.int32)
    for point in obstacle_points:
        y, x = point
        obstacle_grid[y, x] = 0
    return obstacle_grid

def create_clearance_grid(obstacle_grid):
    print("Unique values in obstacle grid:", np.unique(obstacle_grid))

    distance_transform = ndimage.distance_transform_edt(obstacle_grid)

    print("Max distance in distance transform:", np.max(distance_transform))

    max_distance = np.max(distance_transform)
    if max_distance == 0:
        normalized_clearance = np.zeros_like(distance_transform)
    else:
        normalized_clearance = (distance_transform / max_distance) * 100
    
    return normalized_clearance, max_distance

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


def find_obstacle_coords(grid):
    coords = np.where(grid == 1)
    if len(coords[0]) == 0 or len(coords[1]) == 0:
        return None  # No course found

    return list(zip(coords[0], coords[1]))

def edge_polygon_from_course(course_coords):
    return Polygon(course_coords)


def visualize_clearance_grid(clearance_grid, interval=10):
    clearance_grid_resized = cv2.resize(clearance_grid, (clearance_grid.shape[1] // interval, clearance_grid.shape[0] // interval))
    clearance_grid_normalized = cv2.normalize(clearance_grid_resized, None, 0, 255, cv2.NORM_MINMAX)
    clearance_grid_colored = cv2.applyColorMap(clearance_grid_normalized.astype(np.uint8), cv2.COLORMAP_JET)
    return clearance_grid_colored
'''
def visualize_clearance_grid(clearance_grid, interval=10):
    height, width = clearance_grid.shape
    max_clearance_value = np.max(clearance_grid)
    vis_img = np.ones((height * interval, width * interval, 3), np.uint8) * 255

    for y in range(height):
        for x in range(width):
            color_value = int((1 - clearance_grid[y, x] / max_clearance_value) * 255)
            color = (color_value, color_value, color_value)
            cv2.rectangle(vis_img, (x * interval, y * interval), ((x + 1) * interval, (y + 1) * interval), color, -1)

    return vis_img
'''
def visualize_grid(grid, interval=10):
    height, width = grid.shape
    vis_img = np.ones((height * interval, width * interval, 3), np.uint8) * 255

    for y in range(height):
        for x in range(width):
            if grid[y, x] == 1:
                vis_img[y * interval:(y + 1) * interval, x * interval:(x + 1) * interval] = (0, 0, 0)

    return vis_img

def visualize_grid_with_path(grid, interval=10, path=[]):
    vis_img = visualize_grid(grid, interval)

    for (y, x) in path:
        cv2.circle(vis_img, (x * interval + interval // 2, y * interval + interval // 2), interval // 4, (0, 0, 255), -1)

    return vis_img



