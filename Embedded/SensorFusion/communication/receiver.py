import socket
import threading

class Receiver:

    most_recent_data = None
    data_increment = 0
    last_read_data = -1

    def start(self):
        thread = threading.Thread(target=async_receieve, args=(self, ))
        thread.start()

    def get_data(self):
        self.last_read_data = self.data_increment        
        return self.most_recent_data

    def is_ready(self):
        return self.last_read_data < self.data_increment


def async_receieve(receiver):
    with socket.socket() as sock:
        port = 12345
        sock.bind(('', port))

        print("Socket starting, waiting for connection...")

        sock.listen(5)
        conn, addr = sock.accept()

        print("Socket Up and running with a connection from", addr)

        while True:
            received_data = conn.recv(1024).decode()
            receiver.most_recent_data = received_data
            receiver.data_increment += 1

            if(received_data.lower() == "exit"):
                print("Session ended by client.")
                break