import numpy as np
import cv2
from shapely.geometry import Polygon

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


def find_rectangular_course(grid):
    coords = np.where(grid == 1)
    if len(coords[0]) == 0 or len(coords[1]) == 0:
        return None  # No course found

    return list(zip(coords[0], coords[1]))

def edge_polygon_from_course(course_coords):
    return Polygon(course_coords)

def process_grid(binary_grid, interval=1, x_range=(300, 600), y_range=(700, 1100)):
    grid = generate_grid(binary_grid, interval)
    grid = remove_x_from_grid(grid, x_range, y_range, interval)
    return find_rectangular_course(grid)

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



