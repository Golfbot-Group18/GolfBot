import cv2
from Components.MainImageAnalysis import giveMeFrames
from Components.BallDetection import DetectBall, GetBallPoints
from Components.CourseDetection import giveMeBinaryBitch, generate_grid, test_course_boundaries
from Utils.grid_utils import visualize_clearance_grid, visualize_grid, visualize_grid_with_path
from Pathfinding.Pathfinding import a_star_fms_search, calculate_clearance_grid, adjust_goal_for_edge

def main():
    test_course_boundaries()
    frame = giveMeFrames()
    if frame is None:
        return

    balls = DetectBall(frame)
    #balls = GetBallPoints()
    print(f"Balls: {balls}")

    binary_course = giveMeBinaryBitch(frame)
    grid = generate_grid(binary_course, interval=1)
    grid_image = visualize_grid(grid, interval=10)

    start = (int(balls[0][0][1]), int(balls[0][0][0]))
    goal = (int(balls[0][1][1]), int(balls[0][1][0]))
    print(f"Start: {start}")
    print(f"Goal: {goal}")

    max_clearance = 100
    clearance_grid = calculate_clearance_grid(grid, max_clearance)
    clearance_grid_image = visualize_clearance_grid(clearance_grid, interval=10)

    print("Clearance Grid around start and goal:")
    for y in range(max(0, start[0] - 5), min(len(clearance_grid), start[0] + 5)):
        print(clearance_grid[y, max(0, start[1] - 5):min(len(clearance_grid[0]), start[1] + 5)])
    
    for y in range(max(0, goal[0] - 5), min(len(clearance_grid), goal[0] + 5)):
        print(clearance_grid[y, max(0, goal[1] - 5):min(len(clearance_grid[0]), goal[1] + 5)])

    robot_size = 100
    goal_clearance = 5

    path = a_star_fms_search(grid, clearance_grid, start, goal, robot_size, goal_clearance)
    print(f"Path: {path}")

    grid_image_with_path = visualize_grid_with_path(grid, interval=10, path=path)

    cv2.imshow('Grid Visualization', grid_image)
    cv2.imshow('Clearance Grid Visualization', clearance_grid_image)
    cv2.imshow('Grid Visualization with Path', grid_image_with_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
