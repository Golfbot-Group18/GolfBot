from Components.BallDetection import DetectBall
from Components.MainImageAnalysis import giveMeFrames, infiniteCapture
from Components.CourseDetection import generate_grid, visualize_grid, giveMeBinaryBitch, visualize_grid_with_path
from Pathfinding.Pathfinding import a_star_search
import cv2

frame = giveMeFrames()
balls = DetectBall(frame)
print(f"Balls: {balls}")

binary_course = giveMeBinaryBitch(frame)
grid = generate_grid(binary_course, interval=1)
#print(f"Grid: {grid}")
grid_image = visualize_grid(grid, interval=10)

start = (int(balls[0][0][1]), int(balls[0][0][0])) 
goal = (int(balls[0][1][1]), int(balls[0][1][0])) 
print(f"Start: {start}")
print(f"Goal: {goal}")

path = a_star_search(grid, start, goal)
print(f"Path: {path}")
grid_image_with_path = visualize_grid_with_path(grid, interval=10, path=path)

cv2.imshow('Grid Visualization', grid_image)
cv2.imshow('Grid Visualization with Path', grid_image_with_path)
cv2.waitKey(0)
cv2.destroyAllWindows()






