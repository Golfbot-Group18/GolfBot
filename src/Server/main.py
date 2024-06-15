import cv2
from Components.MainImageAnalysis import giveMeFrames
from Components.BallDetection import DetectBall, GetFixedBallPoints
from Camera.Calibration import CalibrateCamera
from Components.CourseDetection import giveMeBinaryBitch
from Components.GridGeneration import generate_grid, visualize_grid, visualize_clearance_grid, visualize_grid_with_path, remove_x_from_grid, find_rectangular_course, edge_polygon_from_course
from Pathfinding.Pathfinding import a_star_fms_search, calculate_clearance_grid, is_goal_in_proximity, is_ball_shiftable

def main():
    frame = giveMeFrames()
    if frame is None:
        return
    frame = CalibrateCamera(frame)

    #balls = DetectBall(frame)
    balls = GetFixedBallPoints()
    print(f"Balls: {balls}")
    binary_course = giveMeBinaryBitch(frame)
    grid = generate_grid(binary_course, interval=1)
    grid_image = visualize_grid(grid, interval=10)
    course_coords = find_rectangular_course(grid)
    for coord in course_coords:
            point = (coord[1]*10, coord[0]*10) # x, y
            cv2.circle(grid_image, point, radius=2, color=(255, 0, 0), thickness=5) 

    max_clearance = 100
    clearance_grid = calculate_clearance_grid(grid, max_clearance)
    clearance_grid_image = visualize_clearance_grid(clearance_grid, interval=10)

    robot_size = 100

    start = (int(balls[0][0][0]), int(balls[0][0][1])) 
    goal = (int(balls[0][1][0]), int(balls[0][1][1]))
    cv2.circle(grid_image, (start[0]*10, start[1]*10), radius=100, color=(0, 0, 255), thickness=50)
    threshold = robot_size/2
    print(f"Start: {start}, Threshold: {threshold}")
    proximity = is_goal_in_proximity(goal, clearance_grid, threshold=threshold)
    print(f"Proximity: {proximity}")

    
    if proximity:
        print("Ball is in proximity to obstacle")
        shiftable, new_goal = is_ball_shiftable(goal, clearance_grid, robot_size, buffer=10) #start is still x, y
        if shiftable:
            print(f"Ball at {goal} can be shifted to new goal at {new_goal}")
        else: 
            print(f"Ball at {goal} cannot be shifted")
            new_goal = goal
    else :
        print("Ball is not in proximity to obstacle")
        new_goal = goal
    
    cv2.circle(grid_image, (new_goal[0]*10, new_goal[1]*10), radius=100, color=(0, 255, 0), thickness=50)
    cv2.imshow('Grid Visualization', grid_image)

    print(f"Start: {start}")
    print(f"Goal: {new_goal}")

    path = a_star_fms_search(grid, clearance_grid, start, new_goal, robot_size)
    print(f"Path: {path}")
    grid_image_with_path = visualize_grid_with_path(grid, interval=10, path=path)
    cv2.imshow('Grid Visualization with Path', grid_image_with_path)

    '''
    y_range = (300, 600)
    x_range = (700, 1100)

    new_grid = remove_x_from_grid(grid, x_range, y_range, interval=1)
    new_grid_image = visualize_grid(new_grid, interval=10)
    course_coords = find_rectangular_course(new_grid)
    print(f"Course Coordinates: {course_coords}")

    for coord in course_coords:
        point = (coord[1]*10, coord[0]*10) # x, y
        cv2.circle(grid_image, point, radius=2, color=(255, 0, 0), thickness=5) 



    start = (int(balls[0][0][1]), int(balls[0][0][0]))
    #goal = (int(balls[0][1][1]), int(balls[0][1][0]))
    #print(f"Start: {start}")
    #print(f"Goal: {goal}")

    max_clearance = 100
    clearance_grid = calculate_clearance_grid(grid, max_clearance)
    clearance_grid_image = visualize_clearance_grid(clearance_grid, interval=10)

    #print("Clearance Grid around start and goal:")
    #for y in range(max(0, start[0] - 5), min(len(clearance_grid), start[0] + 5)):
        #print(clearance_grid[y, max(0, start[1] - 5):min(len(clearance_grid[0]), start[1] + 5)])
    
    #for y in range(max(0, goal[0] - 5), min(len(clearance_grid), goal[0] + 5)):
        #print(clearance_grid[y, max(0, goal[1] - 5):min(len(clearance_grid[0]), goal[1] + 5)])

    robot_size = 100
    goal_clearance = 5

    #path = a_star_fms_search(grid, clearance_grid, start, goal, robot_size, goal_clearance)
    #print(f"Path: {path}")

    #grid_image_with_path = visualize_grid_with_path(grid, interval=10, path=path)
    '''
    #cv2.imshow('Grid Visualization', grid_image)
    #cv2.imshow('Grid with Course', grid_image)
    #cv2.imshow('New grid', new_grid_image)
    #cv2.imshow('Clearance Grid Visualization', clearance_grid_image)
    #cv2.imshow('Grid Visualization with Path', grid_image_with_path)
    cv2.imshow('Frame calibrated', frame)
    #cv2.imshow('Binary course', binary_course)
    cv2.imshow('Clearance Grid Visualization', clearance_grid_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
