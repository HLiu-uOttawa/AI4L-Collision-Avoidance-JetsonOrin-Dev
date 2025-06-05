import socket
import json

HOST = 'localhost'
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    message = {"collision": 1}
    s.sendall(json.dumps(message).encode())

    data = s.recv(1024)
    response = json.loads(data.decode())
    print("Server Response:", response)