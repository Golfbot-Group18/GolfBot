import json
import socket
import cv2
import math
import matplotlib.pyplot as plt
from Components.Communication_module import RobotCommunicator
from Components.RobotDetection import *
from Components.MainImageAnalysis import giveMeFrames, infiniteCapture
from Components.BallDetection import DetectBalls, GetFixedBallPoints
from Camera.Calibration import CalibrateCamera
from Components.CourseDetection import giveMeBinaryBitch, giveMeCourseFramePoints
from Components.GridGeneration import generate_grid, visualize_grid, visualize_clearance_grid, visualize_grid_with_path, remove_x_from_grid, find_obstacle_coords, create_obstacle_grid, create_clearance_grid, analyze_clearance_grid
from Pathfinding.Pathfinding import a_star_fms_search, is_goal_in_proximity, is_ball_shiftable
from Utils.px_conversion import realCoordinates
from Utils.path_conversion import convert_path_to_real_world, generate_vectors_from_path, filter_vectors, calculate_robot_heading, calculate_angle





def send_data(self, current_heading, target_heading, distance, vector_count):
        data = {
            "current_heading": float(current_heading),
            "target_heading": float(target_heading),
            "distance": float(distance),
            "vector_count": int(vector_count)
        }
        message = json.dumps(data)
        self.robot_connection.sendall(message.encode('utf-8'))
        print(f"Sent data to robot: {message}")

def send_heading_to_robot(communicator, current_heading):
    communicator.update_heading(current_heading=current_heading)

def send_distance_to_robot(communicator: RobotCommunicator,position: float ):
     communicator.update_distance(currentPosition=position)

def getDistance(desitnation: tuple, current: tuple)->float:
     
    xDiff = desitnation[1]-current[1]
    yDiff = desitnation[0]-current[0]

    distance = math.sqrt(math.pow(xDiff,2)+math.pow(yDiff,2))

    return distance    

def main():
    

    robot_height_cm = 21.8
    camera_height_cm = 165

    # Robot makes the connection, the server just needs to sit there and look pretty
    communicator = RobotCommunicator(server_ip='0.0.0.0', server_port=12345, confirmation_port=12346)
    communicator.listen_for_robot()
    communicator.listen_for_confirmation()

    while True: 

        # Example of how it could look like: 
        # 'current_heading': 151.4403795216802,
        # 'target_heading': -150.5118841395172
        # 'distance': 37.13531112670898, 
        # 'vector_count': 2, 
        
        #currentHeading = input("Please enter the robots heading: ")
        #targetHeading = input("Please enter the heading to which the robot should turn to: ")
        #distance = input("Please enter the distance the robot should drive: ")
        #amountDirections = input("Please enter how many directions you further want to give the robot: ")

        #if currentHeading == 'f' or targetHeading == 'f' or amountDirections == 'f' or distance == 'f': 
        #     break

        # This is an example of the data sent to a robot
        #communicator.send_data(robot_heading, filtered_vectors[0][1], filtered_vectors[0][0], len(filtered_vectors))
        #communicator.send_data(current_heading=currentHeading, target_heading=targetHeading, distance=distance, vector_count=amountDirections)

        while True:

            #currentHeading = input("Please enter the robots current heading: ")
            currentX = int(input("Input X position: "))
            currentY = int(input("Input Y position: "))
            currentCoordinat = (currentY, currentX)
            


            DestinationCoordinat = (540, 960)
            currentDistance = getDistance(desitnation=DestinationCoordinat, current=currentCoordinat)
            
            # Sending data to the robot
            #send_heading_to_robot(communicator=communicator, current_heading=currentHeading)
            send_distance_to_robot(communicator=communicator, position=currentDistance)
            
            # If the distance sent is less than 3 pixels then it breaks
            if currentDistance < 3:
                 break




if __name__ == "__main__":
    main()