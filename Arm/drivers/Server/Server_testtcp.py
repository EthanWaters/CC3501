import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
from ssl import SOCK_STREAM
import threading
from typing import Any
import queue
import struct
import time


def server_init(received_data, event, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=48):
    s = socket.socket(type=SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen()

    print("Starting a server")
    print(str(HOST)+ ":" + str(PORT))
    
    print("Started reading data")
    

    while True:
        print("check2")
        client_socket, client_address = s.accept()
        data = client_socket.recv(BUFF_SIZE)
        num_float = BUFF_SIZE//4
        received_floats = struct.unpack(f'{num_float}f', data)   
        print("HERE")   
        print("DATA: ", received_floats)
        received_data.put(received_floats)
        event.set()

    print("ENDING ENDING ENDING")
    s.close()


received_data = queue.Queue()
data_available_event = threading.Event()

processing_thread = threading.Thread(target=server_init, args=(received_data, data_available_event, "192.168.0.128", 54321, 48))
processing_thread.start()
                                     
while True:
    something = 1


# "192.168.0.128"
# "10.10.10.42"
