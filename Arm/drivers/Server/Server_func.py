import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
from typing import Any
import queue
import time

def butter_lowpass(cutoff, fs, order):
    nyquist = 0.5*fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a 
    
def butter_lowpass_filter(data, cutoff=1, fs=3, order=4):
    b, a = butter_lowpass(cutoff, fs, order)    
    filtered_data = lfilter(b, a, angle)
    return filtered_data
    


def server_init(received_data, event, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=64):
    s = socket.socket(type=SOCK_DGRAM)
    s.bind((HOST, PORT))
    # s.listen(self.MAX_CONNECTIONS)

    print("Starting a server")
    print(str(HOST)+ ":" + str(PORT))
    
    receiving_data = True
    print("Started reading data")
    # thread = threading.Thread(target=self.get_data)
    # thread.start()
    # s.setblocking(0)
    
    while receiving_data:
        data = s.recv(BUFF_SIZE)
        data = data.decode()
        #print(data)
        if data == "end":
            receiving_data = False
        
        received_data.put(data)
        event.set()
    s.close()


received_data = queue.Queue()
data_available_event = threading.Event()

<<<<<<< HEAD
processing_thread = threading.Thread(target=server_init, args=(received_data, data_available_event, "192.168.0.252", 54321, 1024))
=======
processing_thread = threading.Thread(target=server_init, args=(received_data, data_available_event, "192.168.0.185", 54321, 64))
>>>>>>> 62b31780b4377d3c714efd117908804968d52275
processing_thread.start()
                                     
#server_init(received_data, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=1024)
while True:
    data_available_event.wait()
    data_available_event.clear()
    
    positions = []
    list_type = []
    
    while not received_data.empty():
        positions.append(received_data.get())
    #robot.execute_trajectory_from_poses_and_joints(positions, list_type, DIST_SMOOTHING)
    print(positions)
    time.sleep(1.4)
#thread = threading.Thread(target=server.get_data())
#thread.start()  


