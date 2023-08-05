import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
import time

class Client:
    
    def __init__(self, HOST="127.0.0.1", PORT=54321):
        self.s = socket.socket(type=SOCK_DGRAM)
        self.HOST = HOST
        self.PORT = PORT

    def send_message(self, message):
        self.s.sendto(str(message).encode(), (self.HOST, self.PORT))

client = Client()
count = 0
while count < 20:
    client.send_message([0.1, 0.1, 0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([0.1, 0.1, 0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([0.1, 0.1, 0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([0.1, 0.1, 0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([0.1, 0.1, 0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([0.1, 0.1, 0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([-0.1, -0.1, -0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([-0.1, -0.1, -0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([-0.1, -0.1, -0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([-0.1, -0.1, -0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([-0.1, -0.1, -0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    client.send_message([-0.1, -0.1, -0.1, 0.0, 0.0, 0.0])
    time.sleep(0.01)
    count += 1

client.send_message("end")
