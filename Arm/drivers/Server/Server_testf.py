import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
from typing import Any
import queue
import struct
import time
import numpy as np
from scipy.signal import butter, filtfilt,lfilter


def butter_lowpass(cutoff, fs, order):
    nyquist = 0.5*fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a 
    
def butter_lowpass_filter(data, cutoff=1, fs=3, order=4):
    b, a = butter_lowpass(cutoff, fs, order)    
    filtered_data = lfilter(b, a, angle)
    return filtered_data
    


def server_init(received_data, event, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=48):
    s = socket.socket(type=SOCK_DGRAM)
    s.bind((HOST, PORT))

    print("Starting a server")
    print(str(HOST)+ ":" + str(PORT))
    
    receiving_data = True
    print("Started reading data")
    
    num_points = 500
    count = 0
    file_path = "new_no_filt.txt"
    # fil_b, fil_a = butter_lowpass(cutoff=1.25, fs=3, order=4) 
    # b, a = butter_lowpass(cutoff=1, fs=3, order=4)    
    # with open(file_path, "w") as file:
    while True:
        data = s.recv(BUFF_SIZE)
        if data != b'Hello, UDP!':
            num_float = BUFF_SIZE//4
            data = struct.unpack(f'{num_float}f', data)      
            # filtered_data = lfilter(b, a, data)
            # data = lfilter(fil_b, fil_a, data)
            # data_string = " ".join(map(str, data))
            # file.write(data_string + "\n")  # Add a newline to separate lines

            # file.write(data_string + "\n")  # Add a n
            print("DATA: ", data)
            event.set()
            count += 1
    file.close()
    s.close()


received_data = queue.Queue()
data_available_event = threading.Event()

processing_thread = threading.Thread(target=server_init, args=(received_data, data_available_event, "192.168.0.128", 54321, 48))
processing_thread.start()
                                     
while True:
    print(received_data.get())

# "192.168.0.128"
# "10.10.10.42"
