import socket
import json
import numpy as np

class RobotCommunicater:
    def __init__(self, robot_ip, robot_port, server_ip, server_port):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.connection = None

    def connect_to_robot(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.robot_ip, self.robot_port))
        print("Connected to robot")

    def listen_for_confirmation(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(1)
        self.connection, addr = self.server_socket.accept()
        print(f"Connected to robot confirmation at {addr}")

    def send_data(self, current_heading, target_heading, distance, vector_count):
        data = {
            "current_heading": current_heading,
            "target_heading": target_heading,
            "distance": distance,
            "vector_count": vector_count
        }

        message = json.dumps(data, cls=MyEncoder)
        self.socket.sendall(message.encode('utf-8'))
        print(f"Sent data to robot: {message}")

    def receive_confirmation(self):
        data = self.connection.recv(1024)
        if data:
            confirmation = data.decode('utf-8')
            print(f"Received confirmation from robot: {confirmation}")
            return confirmation
        return None
    
    def close_connections(self):
        if self.connection:
            self.connection.close()
        if self.server_socket:
            self.server_socket.close()
        if self.socket:
            self.socket.close()


class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.float32):
            return float(obj)
        return super(MyEncoder, self).default(obj)