import numpy as np
from Components.Communication_module import RobotCommunicator
from Components.MainImageAnalysis import giveMeFrames
from Components.BallDetection import DetectAllBalls, WhereIsTheOrangeBall, DetectOrangeBall
from Components.RobotDetection import detect_robot, calculate_heading
from Components.CourseDetection import *
from Components.GridGeneration import *
from Components.EggDetection import *
from Pathfinding.Pathfinding import *
from Utils.px_conversion import realCoordinates
import multiprocessing

HOST = '0.0.0.0'
PORT = 12345
REQUEST_PORT = 12346
ROBOT_HEIGHT = 21.8
BALL_HEIGHT = 4
CAMERA_HEIGHT = 165
MIN_CLEARANCE = 1

def look_for_obstacles(frame):
    binary_picture = giveMeBinaryBitch(frame)
    cv2.imshow("Binary picture", binary_picture)
    standard_grid = generate_grid(binary_picture, interval=1)

    obstacle_coords = find_obstacle_coords(standard_grid)
    obstacle_grid = create_obstacle_grid(obstacle_coords, standard_grid.shape)

    clearance_grid, max_distance = create_clearance_grid(obstacle_grid)

    return standard_grid, clearance_grid, max_distance

def initialize_course_processing():
    frame = giveMeFrames()
    standard_grid, clearance_grid, max_distance = look_for_obstacles(frame)
    small_goal_coord, big_goal_coord = giveMeGoalPoints(frame)
    corner_points = giveMeCourseFramePoints(frame)
    return standard_grid, clearance_grid, max_distance, small_goal_coord, big_goal_coord, corner_points
    

def getRobot():
    max_try = 10
    i = 0
    while i < max_try:
        image = giveMeFrames()
        robot_base, robot_tip = detect_robot(image) #this is in (x,y)
        if robot_base is not None and robot_tip is not None:
            robot_base = realCoordinates(ROBOT_HEIGHT, CAMERA_HEIGHT, robot_base) #this is in (x,y)
            robot_tip = realCoordinates(ROBOT_HEIGHT, CAMERA_HEIGHT, robot_tip) #this is in (x,y)
            robot_heading = calculate_heading(robot_base, robot_tip) #this is in degrees
            return robot_base, robot_tip, robot_heading
        i += 1
        
def getBalls():
    max_try = 20
    i = 0
    flattened_ball_positions = []
    orange_ball = None
    while i < max_try:
        image = giveMeFrames()
        ball_positions = DetectAllBalls(image, isolate_white_balls = True) #x, y, radius
        if ball_positions is not None:
            flattened_ball_positions = [(int(ball[0]), int(ball[1])) for ball_array in ball_positions for ball in ball_array]
            for ball in flattened_ball_positions:
                ball = realCoordinates(BALL_HEIGHT, CAMERA_HEIGHT, ball)
                ball = (int(ball[0]), int(ball[1]))
        
        _, orange_ball = WhereIsTheOrangeBall(DetectAllBalls(image), DetectOrangeBall(image))
        if orange_ball is not None:
            print("Orange ball detected.")
            orange_ball = realCoordinates(BALL_HEIGHT, CAMERA_HEIGHT, orange_ball)
            orange_ball = (int(orange_ball[0]), int(orange_ball[1]))
        return flattened_ball_positions, orange_ball
    i += 1  
    
def updated_heading():
    print("Updating heading...")
    max_try = 20
    i = 0
    while i < max_try:
        image = giveMeFrames()
        robot_base, robot_tip = detect_robot(image)
        if robot_base is not None and robot_tip is not None:
            robot_base = realCoordinates(ROBOT_HEIGHT, CAMERA_HEIGHT, robot_base)
            robot_tip = realCoordinates(ROBOT_HEIGHT, CAMERA_HEIGHT, robot_tip)
            robot_heading = calculate_heading(robot_base, robot_tip)
            return int(robot_heading)
        i += 1

def updated_pos_and_heading():
    print("Updating position and heading...")
    max_try = 20
    i = 0
    while i < max_try:
        image = giveMeFrames()
        robot_base, robot_tip = detect_robot(image)
        if robot_base is not None and robot_tip is not None:
            robot_base = realCoordinates(ROBOT_HEIGHT, CAMERA_HEIGHT, robot_base)
            robot_tip = realCoordinates(ROBOT_HEIGHT, CAMERA_HEIGHT, robot_tip)
            robot_heading = calculate_heading(robot_base, robot_tip)

            return (int(robot_base[0]), int(robot_base[1])), int(robot_heading)
        i += 1

def swap_coordinates(point):
    return (point[1], point[0]) 

def get_safe_points(grid, corners, clearance_threshold=40):
    top_left = corners[0]
    top_right = corners[1]
    bottom_right = corners[2]
    bottom_left = corners[3]

    safe_points = []

    # Define quadrant boundaries
    mid_x = (top_left[0] + top_right[0]) // 2
    mid_y = (top_left[1] + bottom_left[1]) // 2

    quadrants = [
        (top_left[0], top_left[1], mid_x, mid_y),  # Top-left quadrant
        (mid_x, top_right[1], top_right[0], mid_y),  # Top-right quadrant
        (bottom_left[0], mid_y, mid_x, bottom_left[1]),  # Bottom-left quadrant
        (mid_x, mid_y, bottom_right[0], bottom_right[1])  # Bottom-right quadrant
    ]

    for (x_start, y_start, x_end, y_end) in quadrants:
        for y in range(y_start, y_end):
            for x in range(x_start, x_end):
                if grid[y][x] >= clearance_threshold:
                    safe_points.append((x, y))
    
    return safe_points

def find_closest_ball_with_path(robot_base, balls, clearance_grid, standard_grid, min_clearance, goal=None, points=None):

    if goal is not None:
        balls.append(goal)

    top_left = points[0]
    top_right = points[1]
    bottom_right = points[2]
    bottom_left = points[3]

    safe_points = get_safe_points(clearance_grid, [top_left, top_right, bottom_right, bottom_left], min_clearance)

    balls.sort(key=lambda x: np.linalg.norm(np.array(x) - np.array(robot_base)))
    
    for closest_ball in balls:
        print(f"Trying to find path to ball at position: {closest_ball}")
        in_corner = any(np.linalg.norm(np.array(closest_ball) - np.array(corner)) < 70 for corner in [top_left, top_right, bottom_right, bottom_left])
        if in_corner:
            print(f"Skipping ball in the corner at position: {closest_ball}")
            continue
        
        # Direct path check
        closest_ball = swap_coordinates(closest_ball) #in (y,x)
        robot_base = swap_coordinates(robot_base) #in (y,x)
        path = a_star_fms_search(standard_grid, clearance_grid, robot_base, closest_ball, min_clearance)

        if len(path) > 0:
            print(f"Found direct path to ball at position (x,y): {swap_coordinates(closest_ball)}") #in (x,y)
            return path, swap_coordinates(closest_ball) #in (y,x)
        
        # Use safe points to navigate
        for safe_point in safe_points:
            safe_path = a_star_fms_search(standard_grid, clearance_grid, robot_base, safe_point, min_clearance)
            if len(safe_path) > 0:
                final_path = a_star_fms_search(standard_grid, clearance_grid, safe_point, closest_ball, min_clearance)
                if len(final_path) > 0:
                    complete_path = safe_path + final_path
                    print(f"Found path to ball via safe point at position (x,y): {swap_coordinates(safe_point)}") #in (x,y)
                    return complete_path, swap_coordinates(closest_ball) #in (y,x)

        print(f"No valid path found to ball at position: {swap_coordinates(closest_ball)}")

    print("No valid path found to any ball.")
    return None, None

def is_direct_path_clear(robot_position, ball_position, grid):
    x0, y0 = robot_position
    x1, y1 = ball_position
    path_clear = True

    # Use Bresenham's line algorithm to check each point along the path
    for x, y in bresenham(x0, y0, x1, y1):
        if grid[y, x] == 1:  # Assuming 1 represents an obstacle
            path_clear = False
            break

    return path_clear

def bresenham(x0, y0, x1, y1):
    """Bresenham's Line Algorithm to generate points on a line between two points."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points

def simplifyPath(path, elipson):
    simplified_path = ramer_douglas_peucker(path, elipson) #path is in (y,x)
    converted_path = []
    for point in simplified_path:
        point = swap_coordinates(point) #swap to (x,y)
        converted_path.append(point)
    return converted_path

def is_front_clear(standard_grid, robot_position, robot_heading, check_distance=100):
    rows, cols = standard_grid.shape
    current_x, current_y = robot_position

    direction_rad = math.radians(robot_heading)
    for distance in range(0, check_distance, 5):  # Check at intervals of 10 units
        check_x = int(current_x + distance * math.cos(direction_rad))
        check_y = int(current_y + distance * math.sin(direction_rad))

        if 0 <= check_x < cols and 0 <= check_y < rows:
            if standard_grid[check_y, check_x] == 1:
                return False
    return True

def find_backup_point(clearance_grid, robot_position, robot_heading, max_backup_distance=30, step_size=10, max_clearance=50):
    rows, cols = clearance_grid.shape
    current_x, current_y = robot_position

    directions = [robot_heading + 180, robot_heading + 135, robot_heading + 225, robot_heading + 150, robot_heading + 210]
    
    best_backup_point = None

    for direction in directions:
        direction_rad = math.radians(direction)
        for distance in range(step_size, max_backup_distance + step_size, step_size):
            check_x = int(current_x + distance * math.cos(direction_rad))
            check_y = int(current_y + distance * math.sin(direction_rad))
            
            if 0 <= check_x < cols and 0 <= check_y < rows:
                clearance = clearance_grid[check_y, check_x]
                if clearance >= max_clearance:
                    return (check_x, check_y)

                if best_backup_point is None or clearance > clearance_grid[best_backup_point[1], best_backup_point[0]]:
                    best_backup_point = (check_x, check_y)
                    
    return best_backup_point

def main():
    print("Starting server... processing course")
    standard_grid, clearance_grid, max_distance, small_goal_coord, large_goal_coord, corner_points = initialize_course_processing()
    safe_points = get_safe_points(clearance_grid, corner_points)
    print(safe_points)
    grid_img = visualize_grid(standard_grid, interval=10)
    clearance_grid_img = visualize_clearance_grid(clearance_grid, interval=10)
    print("Course processed.")

    for point in safe_points:
        point = (point[0]*10, point[1]*10)
        cv2.circle(grid_img, point, 50, (0, 0, 0), 100) 
    cv2.imshow("Grid", grid_img)
    cv2.imshow("Clearance", clearance_grid_img)
    cv2.waitKey(0)

    robot_width = 1
    buffer = 0
    required_clearance = (robot_width / 2 + buffer) / max_distance * 100
    MIN_CLEARANCE = required_clearance

    print("Starting communicator...")
    communicator = RobotCommunicator(HOST, PORT, REQUEST_PORT)
    communicator.start()

    while True:
        robot_base, robot_tip, robot_heading = getRobot() #in (x,y) and degrees
        base = (int(robot_base[0]), int(robot_base[1])) #in (x, y)
        tip = (int(robot_tip[0]), int(robot_tip[1])) #in (x, y)
        heading = int(robot_heading)

        print("Robot detected at: ", base, " with heading: ", heading)

        base_img = (int(base[0]*10), int(base[1]*10)) #in (y, x)
        cv2.circle(grid_img, base_img, 20, (255, 0, 0), 50) #Blue

        tip_img = (int(tip[0]*10), int(tip[1]*10)) #in (y, x)
        cv2.circle(grid_img, tip_img, 20, (0, 255, 0), 50) #Green

        balls, orange_ball = getBalls() #in (x,y)
        print("Number of balls detected: ", len(balls))

        last_trip = False
        if len(balls) == 0 and orange_ball is None:
            print("No balls detected.")
            #here we should initiate a search for the closest goal
            #goal = (int(small_goal_coord[0]), int(small_goal_coord[1])) #in (x,y)
            goal = (int(large_goal_coord[0]), int(large_goal_coord[1])) #in (x,y
            shifted_goal = (goal[0] - 100, goal[1])
            path, closest_ball = find_closest_ball_with_path(base, balls, clearance_grid, standard_grid, MIN_CLEARANCE, goal=shifted_goal, points=corner_points)
            last_trip = True
        elif orange_ball is not None:
            closest_ball = orange_ball  # in (x, y)
            print("Orange ball detected at: ", closest_ball)
            balls.append(closest_ball)  # Ensure the orange ball is included in the ball list
            path, closest_ball = find_closest_ball_with_path(base, balls, clearance_grid, standard_grid, MIN_CLEARANCE, points=corner_points)
   
        else:
            closest_ball_position = min(balls, key=lambda x: np.linalg.norm(np.array(x) - np.array(robot_base)))
            closest_ball = (closest_ball_position[0], closest_ball_position[1])  # in (x, y)
            print("Closest ball detected at: ", closest_ball_position)
            path, closest_ball = find_closest_ball_with_path(base, balls, clearance_grid, standard_grid, MIN_CLEARANCE, points=corner_points)

        
        if path:
            simplified_path = simplifyPath(path, 100)
            print(f"Path to closest ball simplified to {len(simplified_path)} points: {simplified_path}")
            for point in simplified_path:
                center = ((point[0]*10, point[1]*10))
                cv2.circle(grid_img, center, 20, (0, 0, 255), 50)
        else:
            print("No path found to any ball.")
            goal = (int(small_goal_coord[0]), int(small_goal_coord[1])) #in (x,y)
            shifted_goal = (goal[0] - 100, goal[1])
            path, closest_ball = find_closest_ball_with_path(base, balls, clearance_grid, standard_grid, MIN_CLEARANCE, goal=shifted_goal, points=corner_points)
            simplified_path = simplifyPath(path, 100)
            last_trip = True
        
        #cv2.imshow("Grid", grid_img)
        #cv2.waitKey(0)
        
        cv2.imshow("Grid", grid_img)
        cv2.waitKey(0)
        if simplified_path:
            iteration = 1
            print("Sending data to robot...")
            print(f"Current robot position: {robot_base}, Current robot heading: {robot_heading}, Target position: {simplified_path[iteration]}, Number of waypoints: {len(simplified_path) - 1}, Last trip: {last_trip}")
            communicator.send_data(robot_base, robot_heading, simplified_path[iteration], len(simplified_path) - iteration, last_trip=last_trip)
            print("Data sent to robot.")
            while True:
                print("Waiting for request...")
                request = communicator.get_request()
                if request == "check_front_clear":
                    print("Checking front clearance...")
                    _, robot_tip, robot_heading = getRobot()
                    front_clear = is_front_clear(standard_grid, robot_tip, robot_heading)
                    print(f"Front clear: {front_clear}")
                    communicator.send_front_clear(front_clear)

                elif request == "backup_point":
                    print("Finding back up point...")
                    robot_base, robot_heading = updated_pos_and_heading()
                    backup_point = find_backup_point(clearance_grid, robot_base, robot_heading)
                    cv2.circle(grid_img, (backup_point[0]*10, backup_point[1]*10), 20, (255, 0, 0), 50)
                    cv2.imshow("Grid", grid_img)
                    cv2.waitKey(0)
                    print(f"Backup point: {backup_point}")
                    communicator.send_backup_point(backup_point)

                elif request == "update_heading":
                    new_heading = updated_heading()
                    print(f"Updated heading: {new_heading}")
                    communicator.send_updated_heading(new_heading)
                    print("Updated heading sent to robot.")
                elif request == "update_pos_and_heading":
                    new_pos, new_heading = updated_pos_and_heading()
                    print(f"Updated position: {new_pos}, Updated heading: {new_heading}")
                    communicator.send_updated_pos_and_heading(new_pos, new_heading)
                    print("Updated position and heading sent to robot.")
                elif request == "reached_waypoint":
                    iteration += 1
                    new_pos, new_heading = updated_pos_and_heading()
                    print(f"Current robot position: {new_pos}, Current robot heading: {new_heading}, Target position: {simplified_path[iteration]}, Number of waypoints: {len(simplified_path) - iteration}")
                    communicator.send_data(new_pos, new_heading, simplified_path[iteration], len(simplified_path) - iteration)
                elif request == "reached_ball_position":
                    print("Ball reached.")
                    iteration = 1
                    break

if __name__ == "__main__":
    main()

            