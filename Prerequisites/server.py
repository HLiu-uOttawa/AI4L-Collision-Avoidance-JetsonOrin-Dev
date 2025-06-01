import socket
import json

HOST = 'localhost'
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print("Server listening on", HOST, PORT)

    conn, addr = s.accept()
    with conn:
        print("Connected by", addr)
        while True:
            data = conn.recv(1024)
            if not data:
                break
            try:
                received = json.loads(data.decode())
                print("Received:", received)

                response = {
                    "message": "received",
                    "status": "success"
                }
                conn.sendall(json.dumps(response).encode())
            except json.JSONDecodeError:
                conn.sendall(json.dumps({
                    "message": "invalid json",
                    "status": "error"
                }).encode())
