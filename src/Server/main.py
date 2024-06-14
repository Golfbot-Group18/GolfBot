import cv2
from Components.MainImageAnalysis import giveMeFrames
from Components.BallDetection import DetectBall, GetFixedBallPoints
from Components.CourseDetection import giveMeBinaryBitch
from Components.GridGeneration import generate_grid, visualize_grid, visualize_clearance_grid, visualize_grid_with_path, find_inner_box, remove_x_from_grid, find_rectangular_course
from Pathfinding.Pathfinding import a_star_fms_search, calculate_clearance_grid

def main():
    frame = giveMeFrames()
    if frame is None:
        return

    #balls = DetectBall(frame)
    #balls = GetFixedBallPoints()
    #print(f"Balls: {balls}")

    binary_course = giveMeBinaryBitch(frame)
    grid = generate_grid(binary_course, interval=1)
    grid_image = visualize_grid(grid, interval=10)

    y_range = (300, 600)
    x_range = (700, 1100)

    new_grid = remove_x_from_grid(grid, x_range, y_range, interval=1)
    new_grid_image = visualize_grid(new_grid, interval=10)
    course_coords = find_rectangular_course(new_grid)
    print(f"Course Coordinates: {course_coords}")

    for coord in course_coords:
        point = (coord[1]*10, coord[0]*10) # x, y
        cv2.circle(grid_image, point, radius=2, color=(255, 0, 0), thickness=5) 
    
    inner_box = find_inner_box(course_coords, padding=10)
    print("Inner Box Coordinates:", inner_box)

    scaled_inner_box = (
        (inner_box[0][0] * 10, inner_box[0][1] * 10),
        (inner_box[1][0] * 10, inner_box[1][1] * 10)
    )

    # Visualize the inner box on the image
    cv2.rectangle(new_grid_image, scaled_inner_box[0], scaled_inner_box[1], (0, 255, 0), 5)
    cv2.imshow('Detected Inner Box', new_grid_image)
    




    #start = (int(balls[0][0][1]), int(balls[0][0][0]))
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

    cv2.imshow('Grid Visualization', grid_image)
    cv2.imshow('Grid with Course', grid_image)
    cv2.imshow('New grid', new_grid_image)
    cv2.imshow('Clearance Grid Visualization', clearance_grid_image)
    #cv2.imshow('Grid Visualization with Path', grid_image_with_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
