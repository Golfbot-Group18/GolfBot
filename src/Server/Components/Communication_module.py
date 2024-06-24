import socket
import json
import numpy as np

class RobotCommunicator:
    def __init__(self, server_ip, server_port, request_port):
        self.server_ip = server_ip
        self.server_port = server_port
        self.request_port = request_port
        self.server_socket = None
        self.request_socket = None
        self.robot_connection = None
        self.request_connection = None
    

    def listen_for_robot(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(1)
        print(f"Listening for robot connection on {self.server_ip}:{self.server_port}")
        self.robot_connection, addr = self.server_socket.accept()
        print(f"Robot connected from {addr}")

    def listen_for_request(self):
        self.request_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.request_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.request_socket.bind((self.server_ip, self.request_port))
        self.request_socket.listen(1)
        print(f"Listening for robot confirmation on {self.server_ip}:{self.request_port}")
        self.request_connection, addr = self.request_socket.accept()
        print(f"Confirmation connection established with {addr}")
        self.request_connection.settimeout(None)

    def start(self):
        self.listen_for_robot()
        self.listen_for_request()

    def send_updated_heading(self, current_heading):
        data = json.dumps({"current_heading": current_heading})
        self.robot_connection.sendall(data.encode())
        print("Sent robot heading")
    
    def update_position(self, current_position):
        data = json.dumps({"current_position": current_position})
        self.robot_connection.sendall(data.encode())
        print("Sent robot position")
    
    def send_updated_pos_and_heading(self, current_position, current_heading): 
        current_position_list = list(current_position)
        data = {
            "current_heading": current_heading,
            "current_position": current_position_list
        }
        message = json.dumps(data)
        self.robot_connection.sendall(message.encode('utf-8'))
        print(f"Sent heading update to robot: {message}")
    
    def send_front_clear(self, front_clear):
        data = json.dumps({"front_clear": front_clear})
        self.robot_connection.sendall(data.encode())
        print("Sent front clear status to robot")
    
    def send_backup_point(self, backup_point):
        backup_point_list = list(backup_point)
        data = json.dumps({"backup_point": backup_point_list})
        self.robot_connection.sendall(data.encode())
        print("Sent backup point to robot")

    def get_request(self):
        data = self.request_connection.recv(1024)
        request = data.decode('utf-8')
        print(f"Received confirmation from robot: {request}")
        return request
    
    def send_data(self, current_position, current_heading, target_position, vector_count, last_trip=False):
        current_position_list = list(current_position)
        target_position_list = list(target_position)
        data = {
            "current_position": current_position_list,
            "current_heading": float(current_heading),
            "target_position": target_position_list,
            "waypoints_count": int(vector_count),
            "last_trip": False
        }
        
        message = json.dumps(data)
        self.robot_connection.sendall(message.encode('utf-8'))
        print(f"Sent data to robot: {message}")

    def close(self):
        if self.robot_connection:
            self.robot_connection.close()
        if self.server_socket:
            self.server_socket.close()
        if self.request_socket:
            self.request_socket.close()
        if self.request_connection:
            self.request_connection.close()
        print("Connection closed")


class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.float32):
            return float(obj)
        return super(MyEncoder, self).default(obj)