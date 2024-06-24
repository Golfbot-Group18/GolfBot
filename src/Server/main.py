import json
import socket
import cv2
import matplotlib.pyplot as plt
from Components.Communication_module import RobotCommunicator
from Components.RobotDetection import *
from Components.MainImageAnalysis import *
from Components.BallDetection import *
from Camera.Calibration import CalibrateCamera
from Components.CourseDetection import giveMeBinaryBitch, giveMeCourseFramePoints
from Components.GridGeneration import generate_grid, visualize_grid, visualize_clearance_grid, visualize_grid_with_path, remove_x_from_grid, find_obstacle_coords, create_obstacle_grid, create_clearance_grid, analyze_clearance_grid
from Pathfinding.Pathfinding import *
from Utils.px_conversion import realCoordinates
from Utils.path_conversion import *

''' For this we need the 4 corner points!!! - right now they are hardcoded in the function'''
def calculate_homography_matrix(source_points=None):
    print(f"Source points: {source_points}")
    #source_points = np.array([[336, 18], [1605, 35], [342, 955], [1606, 956]], dtype=np.float32) # THESE ARE WRITTEN IN x,y FORMAT
    source_points = np.array(source_points, dtype=np.float32)
    destination_points = np.array([[0, 0], [170, 0], [170, 125], [0, 125]], dtype=np.float32) # THESE ARE WRITTEN IN x,y FORMAT

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
    standard_grid, clearance_grid, max_distance = look_for_obstacles(frame)
    #we also need to place the egg on the course
    #source_points = giveMeCourseFramePoints(frame)
    #we also need to find the points for the 4 corners of the course
    #H, H_inv = calculate_homography_matrix(source_points)
    return standard_grid, clearance_grid, max_distance, #H, H_inv

'''A function to return robot position (center point), heading(relative to itself) and a list of all ball positions'''
def detect_balls(frame):
    balls_result = DetectAllBalls(frame, isolate_white_balls=True)
    if balls_result is not None:
        return balls_result
    return None

def detect_orange_ball(frame):
    _, orange_ball = WhereIsTheOrangeBall(DetectAllBalls(frame), DetectOrangeBall(frame))
    if orange_ball is not None:
        return orange_ball
    return None

def detect_robot(frame):
    robot_base_position, robot_tip_position = Return_robot_position(frame)
    if robot_base_position is not None:
        return robot_base_position, robot_tip_position
    print("Could not find robot position")
    return None, None

def normalize_angle(angle):
    """Normalize angle to be within the -180 to 180 degree range."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def update_robot_heading():
    robot_height_cm = 21.8
    camera_height_cm = 165

    while True:
        image = giveMeFrames()
        robot_base_position, robot_tip_position = detect_robot(image)
        if robot_base_position is not None and robot_tip_position is not None:
            robot_base_position = realCoordinates(robot_height_cm, camera_height_cm, robot_base_position)
            robot_tip_position = realCoordinates(robot_height_cm, camera_height_cm, robot_tip_position)

            robot_heading = Return_robot_heading(robot_base_position, robot_tip_position)
            return robot_heading

def update_robot_position():
    robot_height_cm = 21.8
    camera_height_cm = 164

    while True:
        image = giveMeFrames()
        robot_base_position, _, = detect_robot(image)
        if robot_base_position is not None:
            robot_base_position = realCoordinates(robot_height_cm, camera_height_cm, robot_base_position)
            return robot_base_position # this returns in format (x, y)

def send_distance_to_robot(communicator: RobotCommunicator, remaining_distance, current_heading, current_position):
     communicator.update_distance(distance = remaining_distance, current_heading=current_heading, current_position=current_position)

def getDistance(desitnation: tuple, current: tuple)->float:
     
    xDiff = desitnation[1]-current[1]
    yDiff = desitnation[0]-current[0]

    distance = math.sqrt(math.pow(xDiff,2)+math.pow(yDiff,2))

    return distance

def main():
    frame = giveMeFrames()
    standard_grid, clearance_grid, max_distance, = process_initial_state(frame)

    grid_img = visualize_grid(standard_grid, interval=10)

    robot_width = 80
    buffer = 0
    required_clearance = (robot_width / 2 + buffer) / max_distance * 100
    min_clearance = required_clearance

    robot_height_cm = 21.8
    camera_height_cm = 165

    communicator = RobotCommunicator('0.0.0.0', 12345, 12346)
    communicator.listen_for_robot()
    communicator.listen_for_confirmation()

    while True:
        image = giveMeFrames()
        robot_base_position, robot_tip_position = detect_robot(image)
        ball_positions = detect_balls(image)
        #ball_positions = GetFixedBallPoints()
        orange_ball = detect_orange_ball(image)
        print(f"Robot position (center point): {robot_base_position}    Robot heading point: {robot_tip_position}    Ball positions: {ball_positions}")

        if robot_base_position is None:
            print("No robot detected keep looking")
            continue

        robot_base_position = realCoordinates(robot_height_cm, camera_height_cm, robot_base_position) # this returns in format (x, y)
        robot_base_position = (int(robot_base_position[0]), int(robot_base_position[1])) # this returns in format (x, y)

        robot_base_img = (int(robot_base_position[0] * 10), int(robot_base_position[1] * 10))
        cv2.circle(grid_img, robot_base_img, 20, (255, 0, 0), 50) # Blue

        robot_tip_position = realCoordinates(robot_height_cm, camera_height_cm, robot_tip_position)
        robot_tip_position = (int(robot_tip_position[0]), int(robot_tip_position[1]))

        robot_tip_img = (int(robot_tip_position[0] * 10), int(robot_tip_position[1] * 10))
        cv2.circle(grid_img, robot_tip_img, 20, (0, 255, 0), 50) # Green

        if ball_positions is None:
            print("No balls detected keep looking")
            continue
        
        flattened_ball_positions = [(int(ball[0]), int(ball[1])) for ball_array in ball_positions for ball in ball_array]
        
        if orange_ball is not None:
            closest_ball_position = (int(orange_ball[0]), int(orange_ball[1]))
            print(f"Closest ball is the orange ball at position: {closest_ball_position}")
        else:
            closest_ball_position = min(flattened_ball_positions, key=lambda x: np.linalg.norm(np.array(x) - np.array(robot_base_position)))
            print(f"Closest ball is the ball at position: {closest_ball_position}")

        closest_ball_position = realCoordinates(4, camera_height_cm, closest_ball_position) # this returns in format (x, y)

        closest_ball_updated_position = (int(closest_ball_position[1]), int(closest_ball_position[0])) # this is in format (y, x)

        robot_base_updated_position = (robot_base_position[1], robot_base_position[0]) # this is in format (y, x)   

        robot_heading = round(Return_robot_heading(robot_base_position, robot_tip_position),1) # this is in degrees
        print(f"Robot heading: {robot_heading}")

        path = a_star_fms_search(standard_grid, clearance_grid, robot_base_updated_position, closest_ball_updated_position, min_clearance)

        for point in path:
            center = (int(point[1] * 10), int(point[0] * 10))
            cv2.circle(grid_img, center, 10, (0, 0, 255), 50)

        if len(path) == 0:
            print("No valid path found to closest ball, keep looking.") 
            #better error handling here - not implemented the functions for edge balls either - will do later.
            continue
            

        simplified_path = ramer_douglas_peucker(path, 100)

        for point in simplified_path:
            center = (int(point[1] * 10), int(point[0] * 10))
            cv2.circle(grid_img, center, 10, (0, 255, 255), 50) # Red

        print(f"Simplified path: {simplified_path}")
        converted_path = []
        for point in simplified_path:
            converted_point = (point[1], point[0])  # Swap the coordinates
            converted_path.append(converted_point)

        simplified_path = converted_path

        print(f"Path to closest ball: {simplified_path}")

        cv2.imshow('Standard Grid', grid_img)
        cv2.waitKey(0)
        
        if simplified_path:
            iteration = 1
            print(f"Sending data to robot - Current position: {robot_base_position}, Current heading: {robot_heading}, Target{simplified_path[0]}, Number of waypoints: {len(simplified_path) - 1}")
            communicator.send_data(robot_base_position, robot_heading, simplified_path[iteration], len(simplified_path) - iteration)
            while True:
                print("Waiting for request...")
                confirmation = communicator.receive_confirmation()
                if confirmation == "update_heading":
                    robot_heading = update_robot_heading() # this is in degrees
                    robot_heading = (int(robot_heading))
                    print(f"Updated robot heading: {robot_heading}")
                    send_heading_to_robot(communicator, robot_heading)
                elif confirmation == "update_position_and_heading":
                    robot_base_position = update_robot_position()
                    robot_base_position = (int(robot_base_position[0]), int(robot_base_position[1]))

                    robot_heading = update_robot_heading()
                    robot_heading = (int(robot_heading))

                    send_pos_head_to_robot(communicator, robot_base_position, robot_heading)
                elif confirmation == "reached_waypoint":
                    print("Robot reached a waypoint.")
                    iteration += 1
                    robot_base_position = update_robot_position()
                    robot_base_position = (int(robot_base_position[0]), int(robot_base_position[1]))
                    robot_heading = update_robot_heading()
                    robot_heading = (int(robot_heading))
                    communicator.send_data(robot_base_position, robot_heading, simplified_path[iteration], len(simplified_path) - iteration)
                    continue
                elif confirmation == "reached_goal":
                    print("Robot reached the goal.")
                    iteration = 1
                    break
                        
                        
def send_heading_to_robot(communicator, current_heading):
    print(f"Sending heading to robot: {current_heading}")
    communicator.update_heading(current_heading=current_heading)

def send_pos_to_robot(communicator, current_position):
    communicator.update_position(current_position=current_position)

def send_pos_head_to_robot(communicator, current_position, current_heading):
    communicator.update_position_and_heading(current_position=current_position, current_heading=current_heading)

if __name__ == "__main__":
    main()

