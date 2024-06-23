import socket
import json
import numpy as np

class RobotCommunicator:
    def __init__(self, server_ip, server_port, confirmation_port):
        self.server_ip = server_ip
        self.server_port = server_port
        self.confirmation_port = confirmation_port
        self.server_socket = None
        self.confirmation_socket = None
        self.robot_connection = None
        self.confirmation_connection = None

    def listen_for_robot(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(1)
        print(f"Listening for robot connection on {self.server_ip}:{self.server_port}")
        self.robot_connection, addr = self.server_socket.accept()
        print(f"Robot connected from {addr}")

    def listen_for_confirmation(self):
        self.confirmation_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.confirmation_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.confirmation_socket.bind((self.server_ip, self.confirmation_port))
        self.confirmation_socket.listen(1)
        print(f"Listening for robot confirmation on {self.server_ip}:{self.confirmation_port}")
        self.confirmation_connection, addr = self.confirmation_socket.accept()
        print(f"Confirmation connection established with {addr}")
        self.confirmation_connection.settimeout(None)

    def receive_request(self):
        data = self.robot_connection.recv(1024).decode()
        return data

    def update_heading(self, current_heading):
        data = json.dumps({"current_heading": current_heading})
        self.robot_connection.sendall(data.encode())
        print("Sent robot heading")
    
    def update_position(self, current_position):
        data = json.dumps({"current_position": current_position})
        self.robot_connection.sendall(data.encode())
        print("Sent robot position")

    def receive_confirmation(self):
        data = self.confirmation_connection.recv(1024)
        confirmation = data.decode('utf-8')
        print(f"Received confirmation from robot: {confirmation}")
        return confirmation
    
    def send_data(self, current_position, current_heading, target_heading, distance, vector_count):
        current_position_list = list(current_position)
        data = {
            "current_position": current_position_list,
            "current_heading": float(current_heading),
            "target_heading": float(target_heading),
            "distance": float(distance),
            "waypoints_count": int(vector_count)
        }
        
        message = json.dumps(data)
        self.robot_connection.sendall(message.encode('utf-8'))
        print(f"Sent data to robot: {message}")

    def close(self):
        if self.robot_connection:
            self.robot_connection.close()
        if self.server_socket:
            self.server_socket.close()
        if self.confirmation_socket:
            self.confirmation_socket.close()
        if self.confirmation_connection:
            self.confirmation_connection.close()
        print("Connection closed")


class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.float32):
            return float(obj)
        return super(MyEncoder, self).default(obj)