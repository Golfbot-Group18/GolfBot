import numpy as np
import cv2
from Components.CourseDetection import generate_grid

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
