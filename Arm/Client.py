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
    time.sleep(0.5)
    client.send_message([0.05, -0.1, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.5)
    client.send_message([0.07, -0.3, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.5)
    client.send_message([0.2, -0.4, 0.0, 0.0, 0.0, 0.0])
    client.send_message("end")
    count += 1
client.send_message("end")
