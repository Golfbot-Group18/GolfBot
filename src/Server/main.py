import json
import socket
import cv2
import matplotlib.pyplot as plt
from communication import RobotCommunicator
from Components.RobotDetection import *
from Components.MainImageAnalysis import giveMeFrames, infiniteCapture
from Components.BallDetection import DetectBalls, GetFixedBallPoints
from Camera.Calibration import CalibrateCamera
from Components.CourseDetection import giveMeBinaryBitch
from Components.GridGeneration import generate_grid, visualize_grid, visualize_clearance_grid, visualize_grid_with_path, remove_x_from_grid, find_obstacle_coords, create_obstacle_grid, create_clearance_grid, analyze_clearance_grid
from Pathfinding.Pathfinding import a_star_fms_search, is_goal_in_proximity, is_ball_shiftable
from Utils.px_conversion import calculate_scale_factor_from_ball
from Utils.path_conversion import convert_path_to_real_world, generate_vectors_from_path, filter_vectors, calculate_robot_heading, calculate_angle

''' For this we need the 4 corner points!!! - right now they are hardcoded in the function'''
def calculate_homography_matrix(source_points=None):
    source_points = np.array([[336, 18], [1605, 35], [342, 955], [1606, 956]], dtype=np.float32) # THESE ARE WRITTEN IN x,y FORMAT
    
    destination_points = np.array([[0, 0], [170, 0], [0, 125], [170, 125]], dtype=np.float32) # THESE ARE WRITTEN IN x,y FORMAT

    H, status = cv2.findHomography(source_points, destination_points)
    H_inv = np.linalg.inv(H)
    return H, H_inv

'''This function is used to convert the image coordinates to real world coordinates using the homography matrix'''
def image_to_real_world(x, y, H):
    image_point = np.array([[x, y]], dtype='float32')
    image_point = np.array([image_point])
    real_world_point = cv2.perspectiveTransform(image_point, H)
    return real_world_point[0][0]

'''This function is used to convert the real world coordinates to image coordinates using the homography matrix - I don't know if we need this'''
def real_world_to_image(x, y, H_inv):
    real_world_point = np.array([[x, y]], dtype='float32')
    real_world_point = np.array([real_world_point])
    image_point = cv2.perspectiveTransform(real_world_point, H_inv)
    return image_point[0][0]

'''This function is used to locate obstacles in a picture'''
def look_for_obstacles(frame):
    binary_picture = giveMeBinaryBitch(frame)
    standard_grid = generate_grid(binary_picture, interval=1)

    obstacle_coords = find_obstacle_coords(standard_grid)
    obstacle_grid = create_obstacle_grid(obstacle_coords, standard_grid.shape)

    clearance_grid, max_distance = create_clearance_grid(obstacle_grid)
    return standard_grid, clearance_grid, max_distance

'''We use this function to indicate the initial process is done'''
def wait_for_start():
    input("Initial processing done - Enter to start...")

'''This function processes the initial state of the image'''
def process_initial_state(frame):
    standard_grid, clearance_grid, max_distance = look_for_obstacles()
    #we also need to place the egg on the course

    #we also need to find the points for the 4 corners of the course
    H, H_inv = calculate_homography_matrix()
    return standard_grid, clearance_grid, max_distance, H, H_inv

'''A function to return robot position (center point), heading(relative to itself) and a list of all ball positions'''
def detect_robot_and_balls(frame):
    robot_position, robot_heading = Return_heading_position()
    ball_positions, orange_ball_index = DetectBalls(frame),
    return robot_position, robot_heading, ball_positions, orange_ball_index

def calculate_radial_distance(robot_x, robot_y, camera_center_x=540, camera_center_y=960, camera_height=100, robot_height=10):
    x_diff = robot_x - camera_center_x
    y_diff = robot_y - camera_center_y
    
    r = math.sqrt(x_diff**2 + y_diff**2)
    
    delta_f = camera_height - robot_height
    
    correction_factor = delta_f / camera_height
    
    corrected_r = r * correction_factor
    
    if r != 0:
        corrected_x = camera_center_x + (x_diff / r) * corrected_r
        corrected_y = camera_center_y + (y_diff / r) * corrected_r
    else:
        corrected_x = camera_center_x
        corrected_y = camera_center_y
    
    return corrected_x, corrected_y

def main():
    frame = giveMeFrames()
    standard_grid, clearance_grid, max_distance, H, H_inv, = process_initial_state(frame)
    robot_width = 1  # side_lengths[0][0] * 2
    buffer = 0
    required_clearance = (robot_width / 2 + buffer) / max_distance * 100
    min_clearance = required_clearance
    wait_for_start()

    communicator = RobotCommunicator(robot_ip='0.0.0.0', robot_port=12345, server_ip='0.0.0.0', server_port=12346)
    communicator.connect_to_robot()
    communicator.listen_for_confirmation()
    if communicator.connection is None:
        print("Could not connect to robot")
        ## need better error handling here
        return
        

    while True:
        image = giveMeFrames()
        robot_position, robot_heading, ball_positions, orange_ball_index = detect_robot_and_balls(image)
        print(f"Robot position (center point): {robot_position}    Robot heading (in degrees): {robot_heading}    Ball positions: {ball_positions}")

        if robot_position or robot_heading is None:
            print("No robot detected keep looking")
            continue
        robot_position = calculate_radial_distance(robot_position[0], robot_position[1]) #we get this in x, y format
        if ball_positions is None:
            print("No balls detected keep looking")
            continue
        
        if orange_ball_index is not None:
            closest_ball = ball_positions[orange_ball_index]
            print(f"Closest ball is the orange ball at position: {closest_ball}")
        else:
            closest_ball = min(ball_positions, key=lambda x: np.linalg.norm(np.array(x) - np.array(robot_position)))
            print(f"Closest ball is the ball at position: {closest_ball}")

        path = a_star_fms_search(standard_grid, clearance_grid, robot_position, closest_ball, min_clearance)

        if path is None:
            print("No valid path found to closest ball, keep looking.") 
            #better error handling here - not implemented the functions for edge balls either.
            continue
        
        path_cm = [image_to_real_world(p[1], p[0], H) for p in path]
        print(f"Path in cm: {path_cm}")

        vectors = generate_vectors_from_path(path_cm)

        relative_vectors = [(distance, (angle - robot_heading) % 360) for distance, angle in vectors]
        filtered_vectors = filter_vectors(relative_vectors)
        print(f"Filtered vectors: {filtered_vectors}")

        if filtered_vectors:
            print(f"Sending data to robot - Current heading: {robot_heading}, Target heading: {filtered_vectors[0][1]}, Distance: {filtered_vectors[0][0]}, Number of waypoints: {len(filtered_vectors)}")
            communicator.send_data(robot_heading, filtered_vectors[0][1], filtered_vectors[0][0], len(filtered_vectors))

            while True:
                image = giveMeFrames()
                robot_position, robot_heading, _ = detect_robot_and_balls(image)
                robot_position = calculate_radial_distance(robot_position[0], robot_position[1]) #we get this in x, y format
                print(f'Robot position: {robot_position}    Robot heading: {robot_heading}')

                send_heading_to_robot(communicator, robot_heading)

                confirmation = communicator.receive_confirmation()
                print(f"Confirmation from robot: {confirmation}")

                if confirmation == "reached":

                    if np.linalg.norm(np.array(robot_position) - np.array(closest_ball)) < 10:
                        print("Robot reached the ball.")
                        break

                    path = a_star_fms_search(standard_grid, clearance_grid, robot_position, closest_ball, 0)
                    if path is None:
                        print("No valid path found, keep looking.")
                        break

                    path_cm = [image_to_real_world(p[1], p[0], H) for p in path]
                    vectors = generate_vectors_from_path(path_cm)
                    relative_vectors = [(distance, (angle - robot_heading) % 360) for distance, angle in vectors]
                    filtered_vectors = filter_vectors(relative_vectors)

                    if filtered_vectors:
                        communicator.send_data(robot_position, robot_heading, filtered_vectors[0][1], filtered_vectors[0][0], len(filtered_vectors))
                    else:
                        print("Filtered vectors are empty, keep looking.")
                        break

def send_heading_to_robot(communicator, current_heading):
    communicator.send_data(current_heading=current_heading)
                







        


    ##First we need to process the initial state of the course
    ##This includes finding the obstacles and generating the grid
    ## We'll also need to calculate the homography matrix
    ## We'll make a function for this - process_initial_state()

    ##Then we'll wait for the user to press enter - this indicates that the initial state has been processed

    ##Then we'll start the main loop
    ##In the main loop we'll look for the robot and the balls
    ##We'll calculate the heading of the robot
    ##We'll calculate the path to the closest ball
    ##We'll convert the path to vectors and filter them to get the most important ones
    ##if the path contains multiple vectors, we'll send the first one to the robot
        ##Here we start a new loop
        ## In this loop we'll constantly update the robot with it's current heading
        # We'll listen for the robot to confirm that it has reached the first waypoint
            # Here we'll check if it indeed has reached the first waypoint
        ## When the robot reaches the first waypoint, we'll calculate the path again
        ## We'll convert the new path and convert it to vectors and send the first one to the robot
        ## We'll keep doing this until the robot reaches the ball
        ## (When the robot reaches the ball, it itself will initiate a protocol to pick up the ball)
        ## This loop will break when the robot reaches the goal position
    ##When the robot reaches the goal position, we'll start the main loop again

    

if __name__ == "__main__":
    main()

'''
def main():

    H, H_inv = calculate_homography_matrix()
    standard_grid, clearance_grid, max_distance = look_for_obstacles()
    grid_image = visualize_grid(standard_grid, interval=10)
    wait_for_start()

    frame = giveMeFrames()
    frame = CalibrateCamera(frame)
    robot_contour = DetectRobot(frame)
    while robot_contour is None:
        print("No robot detected keep looking")
        frame = giveMeFrames()
        if frame is None:
            continue
        robot_contour = DetectRobot(frame)

    side_lengths = CalculateRobotTriangle(robot_contour)
    robot_heading_points = CalculateRobotHeading(robot_contour)
    print(f"Robot heading points: {robot_heading_points}")
    if robot_heading_points is None:
        print("Could not calculate robot heading")
        return

    robot_base = np.array(robot_heading_points[0], dtype=int)
    robot_tip = np.array(robot_heading_points[1], dtype=int)

    robot_base_real = image_to_real_world(robot_base[0], robot_base[1], H)
    robot_tip_real = image_to_real_world(robot_tip[0], robot_tip[1], H)

    print (f"Robot base image: {robot_base}    Robot base real: {robot_base_real}")
    print (f"Robot tip image: {robot_tip}    Robot tip real: {robot_tip_real}")

    robot_heading_vector = (robot_tip_real[0] - robot_base_real[0], robot_tip_real[1] - robot_base_real[1])
    robot_heading_angle = calculate_angle(robot_heading_vector)
    print(f"Robot heading angle: {robot_heading_angle}")
    #robot_heading = calculate_robot_heading((robot_base[1], robot_base[0]), (robot_tip[1], robot_tip[0]))
    #robot_heading_real = calculate_robot_heading((robot_base_real[1], robot_base_real[0]), (robot_tip_real[1], robot_tip_real[0]))
    #print(f"Robot heading image: {robot_heading}    Robot heading real: {robot_heading_real}")

    cv2.circle(grid_image, (robot_base[0] * 10, robot_base[1] * 10), 20, (255, 0, 0), 50) # Blue
    cv2.circle(grid_image, (robot_tip[0] * 10, robot_tip[1] * 10), 20, (0, 255, 0), 50) # Green

    robot_width = 1  # side_lengths[0][0] * 2
    buffer = 0
    required_clearance = (robot_width / 2 + buffer) / max_distance * 100
    min_clearance = required_clearance

    balls, orange_ball_index = DetectBalls(frame)
    print(f"Ball info image: {balls}")
    ##while balls.size == 0:
    ##  print("No balls detected, keep looking")
    ##  frame = giveMeFrames()
    ##  if frame is None:
    ##      continue
    ##  balls = GetFixedBallPoints()

    ball_coords = [(int(ball[1]), int(ball[0])) for sublist in balls for ball in sublist]
    #ball_coords = [(int(ball[1]), int(ball[0])) for sublist in balls for ball in sublist]
    print(f"Ball coordinates on image: {ball_coords}")

    robot_base_coord = (int(robot_base[1]), int(robot_base[0]))
    print(f"Robot base coord on Image: {robot_base_coord}")
    robot_tip_coord = (int(robot_tip[1]), int(robot_tip[0]))
    print(f"Robot tip coord on Image: {robot_tip_coord}")

    shortest_path = None
    shortest_distance = float('inf')

    for ball_coord in ball_coords:
        print(f"Planning path from image robot tip {robot_tip_coord} to image ball {ball_coord}")
        
        if 0 <= ball_coord[0] < standard_grid.shape[0] and 0 <= ball_coord[1] < standard_grid.shape[1]:
            path = a_star_fms_search(standard_grid, clearance_grid, robot_tip_coord, ball_coord, min_clearance)
            if path:
                path_distance = sum([np.linalg.norm(np.array(path[i]) - np.array(path[i - 1])) for i in range(1, len(path))])
                if path_distance < shortest_distance:
                    shortest_distance = path_distance
                    shortest_path = path
            else:
                print(f"Ball image coordinate {ball_coord} is out of bounds.")

    if len(shortest_path) != 0:
        for point in shortest_path:
            cv2.circle(grid_image, (point[1] * 10, point[0] * 10), 10, (0, 0, 255), 30)

    shortest_path_cm = [image_to_real_world(p[1], p[0], H) for p in shortest_path]

    vectors_image = generate_vectors_from_path(shortest_path)
    filtered_vectors_image = filter_vectors(vectors_image)
    print(f"Filtered vectors on image: {filtered_vectors_image}")

    # Generate vectors in real-world coordinates and adjust angles relative to robot heading
    vectors_real = generate_vectors_from_path(shortest_path_cm)
    relative_vectors_real = [(distance, (angle - robot_heading_angle) % 360) for distance, angle in vectors_real]
    filtered_vectors_real = filter_vectors(relative_vectors_real)
    print(f"Filtered vectors real: {filtered_vectors_real}")

    cv2.imshow('Standard Grid', grid_image)
    cv2.imshow('Clearance Grid', visualize_clearance_grid(clearance_grid, interval=10))
    cv2.imshow('Path', visualize_grid_with_path(standard_grid, interval=10, path=shortest_path))
    cv2.waitKey(0)
    start_server(filtered_vectors_real)
    cv2.destroyAllWindows()

'''

'''I've tried making a simple main function that will be used to run the entire program. It will first look for obstacles on the course, then wait for the user to place the robot on the course. After that, it will start the main loop where it will look for the robot and balls, calculate the path to the closest ball and then send the path to the robot using a socket connection'''
'''
def start_server(vectors, robot_heading, host='0.0.0.0', port=65432):
    data = {
        'robot_heading': robot_heading,
        'vectors': vectors
    }
    data_json = json.dumps(data)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen(1)
        print(f"Server listening on {host}:{port}")

        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            conn.sendall(data_json.encode('utf-8'))
            print("Vectors sent")

def look_for_obstacles():
    while True:
        frame = giveMeFrames()
        if frame is None:
            continue
        frame = CalibrateCamera(frame)
        binary_course = giveMeBinaryBitch(frame)
        standard_grid = generate_grid(binary_course, interval=1)

        obstacle_coords = find_obstacle_coords(standard_grid)
        obstacle_grid = create_obstacle_grid(obstacle_coords, standard_grid.shape)
        clearance_grid, max_distance = create_clearance_grid(obstacle_grid)
        return standard_grid, clearance_grid, max_distance

def wait_for_start():
    input("Place the robot on the course and press Enter to start...")


def main ():
    #infiniteCapture()
    standard_grid, clearance_grid, max_distance = look_for_obstacles()
    grid_image = visualize_grid(standard_grid, interval=10)
    wait_for_start()

    frame = giveMeFrames()
    frame = CalibrateCamera(frame)
    robot_contour = DetectRobot(frame)
    while robot_contour is None:
        print("No robot detected keep looking")
        frame = giveMeFrames()
        if frame is None:
            continue
        frame = CalibrateCamera(frame)
        robot_contour = DetectRobot(frame)
    
    side_lengths = CalculateRobotTriangle(robot_contour)
    robot_heading_points = CalculateRobotHeading(robot_contour)
    print(f"Robot heading points: {robot_heading_points}")
    if robot_heading_points is None:
        print("Could not calculate robot heading")
        return
    robot_base = np.array(robot_heading_points[0], dtype=int)
    robot_tip = np.array(robot_heading_points[1], dtype=int)
    robot_heading = calculate_robot_heading((robot_base[1], robot_base[0]), (robot_tip[1], robot_tip[0]))
    print(f"Robot heading: {robot_heading}")
    cv2.circle(grid_image, (robot_base[0] * 10, robot_base[1] * 10), 20, (255, 0, 0), 50) #What is this color? Blue
    cv2.circle(grid_image, (robot_tip[0] * 10, robot_tip[1] * 10), 20, (0, 255, 0), 50) #What is this color? Green
    cv2.circle(grid_image, (0, 0), 20, (0, 255, 0), 50 )

    robot_width = 1 # side_lengths[0][0] * 2
    buffer = 0
    required_clearance = (robot_width / 2 + buffer) / max_distance * 100
    min_clearance = required_clearance

    balls, orange_index = DetectBalls(frame)
    while balls.size == 0:
        print("No balls detected keep looking")
        frame = giveMeFrames()
        if frame is None:
            continue
        frame = CalibrateCamera(frame)
        balls = DetectBalls(frame)

    scale_factor = calculate_scale_factor_from_ball(4, balls)
    robot_base_coord = (int(robot_base[1]), int(robot_base[0]))
    robot_tip_coord = (int(robot_tip[1]), int(robot_tip[0]))
    print(f"Ball points: {balls}")

    ball_coords = [(int(ball[1]), int(ball[0])) for sublist in balls for ball in sublist]
    print(f"Ball coordinates: {ball_coords}")

    shortest_path = None
    shortest_distance = float('inf')

    for ball_coord in ball_coords:
        print(f"Planning path from robot tip {robot_tip_coord} to ball {ball_coord}")
        if 0 <= ball_coord[0] < standard_grid.shape[0] and 0 <= ball_coord[1] < standard_grid.shape[1]:
            path = a_star_fms_search(standard_grid, clearance_grid, robot_base_coord, ball_coord, min_clearance)
            if path:
                path_distance = sum([np.linalg.norm(np.array(path[i]) - np.array(path[i - 1])) for i in range(1, len(path))])
                if path_distance < shortest_distance:
                    shortest_distance = path_distance
                    shortest_path = path
            else:
                print(f"Ball coordinate {ball_coord} is out of bounds.")

    if len(shortest_path) == 0:
        print("No valid path found to any ball, keep looking.")
    elif len(shortest_path) != 0:
        for point in path:
            cv2.circle(grid_image, (point[1] * 10, point[0] * 10), 10, (0, 0, 255), 30)
        
        shortest_path_cm = convert_path_to_real_world(shortest_path, scale_factor)
        vectors = generate_vectors_from_path(shortest_path_cm)
        filtered_vectors = filter_vectors(vectors)

        print(f"Filtered vectors: {filtered_vectors}")

        cv2.imshow('Standard Grid', grid_image)
        cv2.imshow('Clearance Grid', visualize_clearance_grid(clearance_grid, interval=10))
        cv2.waitKey(0)
        start_server(filtered_vectors, robot_heading)
        cv2.destroyAllWindows()
'''
'''
def main():
    standard_grid, clearance_grid, max_distance = look_for_obstacles()
    grid_image = visualize_grid(standard_grid, interval=10)
    wait_for_start()
    while True:
        frame = giveMeFrames()
        robot_contour = DetectRobot(frame)
        while robot_contour is None:
            print("No robot detected keep looking")
            frame = giveMeFrames()
            if frame is None:
                continue
            frame = CalibrateCamera(frame)
            robot_contour = DetectRobot(frame)
        if robot_contour is None:
            print("No robot detected keep looking")
            continue
            
        side_lengths = CalculateRobotTriangle(robot_contour)
        robot_heading_points = CalculateRobotHeading(robot_contour)
        print(f"Robot heading points: {robot_heading_points}")
        if robot_heading_points is None:
            print("Could not calculate robot heading")
            continue
        
        robot_base = np.array(robot_heading_points[0], dtype=int)
        robot_tip = np.array(robot_heading_points[1], dtype=int)
        robot_heading = calculate_robot_heading((robot_base[1], robot_base[0]), (robot_tip[1], robot_tip[0]))
        print(f"Robot heading: {robot_heading}")
        cv2.circle(grid_image, (robot_base[0] * 10, robot_base[1] * 10), 20, (255, 0, 0), 50) #What is this color? Blue
        cv2.circle(grid_image, (robot_tip[0] * 10, robot_tip[1] * 10), 20, (0, 255, 0), 50) #What is this color? Green
        cv2.circle(grid_image, (0, 0), 20, (0, 255, 0), 50 )

        robot_width = 1 # side_lengths[0][0] * 2
        buffer = 0
        required_clearance = (robot_width / 2 + buffer) / max_distance * 100
        min_clearance = required_clearance

        balls, orange_index = DetectBalls(frame)
        while balls.size == 0:
            print("No balls detected keep looking")
            frame = giveMeFrames()
            if frame is None:
                continue
            frame = CalibrateCamera(frame)
            balls = DetectBalls(frame)

        scale_factor = calculate_scale_factor_from_ball(4.4, balls)
        robot_tip_coord = (int(robot_tip[1]), int(robot_tip[0]))
        print(f"Ball points: {balls}")

        ball_coords = [(int(ball[1]), int(ball[0])) for sublist in balls for ball in sublist]
        print(f"Ball coordinates: {ball_coords}")

        shortest_path = None
        shortest_distance = float('inf')

        for ball_coord in ball_coords:
            print(f"Planning path from robot tip {robot_tip_coord} to ball {ball_coord}")
            if 0 <= ball_coord[0] < standard_grid.shape[0] and 0 <= ball_coord[1] < standard_grid.shape[1]:
                path = a_star_fms_search(standard_grid, clearance_grid, robot_tip_coord, ball_coord, min_clearance)
                if path:
                    path_distance = sum([np.linalg.norm(np.array(path[i]) - np.array(path[i - 1])) for i in range(1, len(path))])
                    if path_distance < shortest_distance:
                        shortest_distance = path_distance
                        shortest_path = path
            else:
                print(f"Ball coordinate {ball_coord} is out of bounds.")

        if shortest_path is None:
            print("No valid path found to any ball, keep looking.")
            continue
        else: 
            for point in path:
                cv2.circle(grid_image, (point[1] * 10, point[0] * 10), 10, (0, 0, 255), 30)

        #print(f"Shortest path: {shortest_path}")
        shortest_path_cm = convert_path_to_real_world(shortest_path, scale_factor)
        vectors = generate_vectors_from_path(shortest_path_cm)
        #print(f"Vectors: {vectors}")
        filtered_vectors = filter_vectors(vectors)

        print(f"Filtered vectors: {filtered_vectors}")

        if filtered_vectors:
            first_vector = filtered_vectors[0]
            turn_angle = first_vector[1] - robot_heading
            if turn_angle > 180:
                turn_angle -= 360
            elif turn_angle < -180:
                turn_angle += 360

        print(f"Turn robot by {turn_angle} degrees.")
        print(f"Move robot forward by {first_vector[0]} cm.")

        robot_heading = first_vector[1]
        break
        ##start_server(filtered_vectors, robot_heading)
        #cv2.imshow('Binary Course', binary_course)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.imshow('Standard Grid', grid_image)
    cv2.imshow('Clearance Grid', visualize_clearance_grid(clearance_grid, interval=10))
    cv2.destroyAllWindows()
    
'''

