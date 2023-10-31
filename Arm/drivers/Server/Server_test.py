import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
from typing import Any
import queue
import time


def server_init(received_data, event, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=64):
    s = socket.socket(type=SOCK_DGRAM)
    s.bind((HOST, PORT))

    print("Starting a server")
    print(str(HOST)+ ":" + str(PORT))
    
    receiving_data = True
    print("Started reading data")
    
    while receiving_data:
        data = s.recv(BUFF_SIZE)
        if data != b'Hello, UDP!':
            print(data)
            if data == "end":
                receiving_data = False
            received_data.put(data)
            event.set()
    s.close()


received_data = queue.Queue()
data_available_event = threading.Event()

processing_thread = threading.Thread(target=server_init, args=(received_data, data_available_event, "192.168.0.128", 54321, 64))
processing_thread.start()
                                     
while True:
    # print(received_data.get())
    print_he = "HI"

# "192.168.0.128"
# "10.10.10.42"
