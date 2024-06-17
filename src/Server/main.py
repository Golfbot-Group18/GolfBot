import json
import socket
import cv2
import matplotlib.pyplot as plt
from Components.RobotDetection import *
from Components.MainImageAnalysis import giveMeFrames, infiniteCapture
from Components.BallDetection import DetectBall, GetFixedBallPoints
from Camera.Calibration import CalibrateCamera
from Components.CourseDetection import giveMeBinaryBitch
from Components.GridGeneration import generate_grid, visualize_grid, visualize_clearance_grid, visualize_grid_with_path, remove_x_from_grid, find_obstacle_coords, create_obstacle_grid, create_clearance_grid, analyze_clearance_grid
from Pathfinding.Pathfinding import a_star_fms_search, calculate_clearance_grid, is_goal_in_proximity, is_ball_shiftable
from Utils.px_conversion import calculate_scale_factor_from_ball
from Utils.path_conversion import convert_path_to_real_world, generate_vectors_from_path, filter_vectors

def start_server(vectors, host='0.0.0.0', port=65432):
    vectors_json = json.dumps(vectors)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen(1)
        print(f"Server listening on {host}:{port}")

        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            conn.sendall(vectors_json.encode('utf-8'))
            print("Vectors sent")

def main():
    #infiniteCapture()
    frame = giveMeFrames()
    if frame is None:
        return
    frame = CalibrateCamera(frame)
    balls = DetectBall(frame)
    print(f"Ball points: {balls}")

    robot_contour = DetectRobot(frame)
    if robot_contour is not None:
        side_lengths = CalculateRobotTriangle(robot_contour)
        robot_width = side_lengths[0][0] * 2
        print(f"Robot side lengths: {side_lengths}")

    else:
        print("No robot detected")

    scale_factor = calculate_scale_factor_from_ball(4.4, balls)

    binary_course = giveMeBinaryBitch(frame)
    standard_grid = generate_grid(binary_course, interval=1)
    grid_image = visualize_grid(standard_grid, interval=10)
    
    obstacle_coords = find_obstacle_coords(standard_grid)
    for coord in obstacle_coords:
        point = (coord[1] * 10, coord[0] * 10)
        cv2.circle(grid_image, point, 5, (0, 0, 255), 10)
    if(len(obstacle_coords) > 0):
        print(f"Found obstacles with {len(obstacle_coords)} points")
    else:
        print("No obstacles found")
    obstacle_grid = create_obstacle_grid(obstacle_coords, standard_grid.shape)
    
    clearance_grid, max_distance = create_clearance_grid(obstacle_grid)

    clearance_image = visualize_clearance_grid(clearance_grid, interval=10)
    analyze_clearance_grid(clearance_grid) 

    start = (int(balls[0][0][1]), int(balls[0][0][0]))
    end = (int(balls[0][1][1]), int(balls[0][1][0]))
    robot_size = robot_width
    buffer = 10
    required_clearance = (robot_size / 2 + buffer) / max_distance * 100
    min_clearance = required_clearance

    print(f"Clearance at start point ({start}): {clearance_grid[start[0], start[1]]}")
    print(f"Clearance at goal point ({end}): {clearance_grid[end[0], end[1]]}")
    print(f"Required normalized clearance: {required_clearance}")

    cv2.circle(grid_image, (start[1] * 10, start[0] * 10), 20, (0, 255, 0), 50) #What is this color? Green
    cv2.circle(grid_image, (end[1] * 10, end[0] * 10), 20, (255, 0, 0), 50) #What is this color? Blue

    path = a_star_fms_search(standard_grid, clearance_grid, start, end, min_clearance)
    print(f"Path found: {path}")
    if path:
        for point in path:
            cv2.circle(grid_image, (point[1] * 10, point[0] * 10), 10, (0, 0, 255), 30)
    
    path = convert_path_to_real_world(path, scale_factor)
    print(f"Path in real world: {path}")
    vectors = generate_vectors_from_path(path)
    print(f"Vectors: {vectors}")
    filtered_vectors = filter_vectors(vectors)
    print(f"Filtered vectors: {filtered_vectors}")
    #start_server(filtered_vectors)
    
    cv2.imshow('Binary Course', binary_course)
    cv2.imshow('Standard Grid', grid_image)
    cv2.imshow('Clearance Grid', clearance_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
