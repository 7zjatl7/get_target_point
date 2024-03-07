import socket
import threading

class SocketServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn = None
        self.addr = None

    def start_server(self):

        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            print(f"Server listening on {self.host}:{self.port}")
            while True:
                conn, addr = self.server_socket.accept()
                print(f"Connection established with {addr}")
                client_thread = threading.Thread(target=self.handle_client, args=(conn,))
                client_thread.start()
        except socket.error as e:
            print(f"Socket error: {e}")
        except Exception as e:
            print(f"An exception occurred: {e}")

    def handle_client(self, conn):
        try:
            while True:
                data = conn.recv(1024).decode('utf-8')
                if not data:
                    break
                print(f"Received data: {data}")
        finally:
            conn.close()
