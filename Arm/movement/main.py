"""PI to ARM code. Ethan, Stuart & Lachlan"""

from pickle import bytes_types
from pyniryo import *
# from pynput import keyboard
import struct
import threading
import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
from typing import Any
import queue
import time
import numpy as np
from scipy.signal import butter, filtfilt,lfilter



WIFI_IP_ADDRESS = "192.168.0.177"
DIST_SMOOTHING = 0.05
HOST = "192.168.0.128"
PORT = 54321
BUFF_SIZE=48
SPEED = 500
PORT_ROBOT = 4001


def butter_lowpass(cutoff, fs, order):
    nyquist = 0.5*fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a 
    
def butter_lowpass_filter(data, cutoff=1, fs=3, order=4):
    b, a = butter_lowpass(cutoff, fs, order)    
    filtered_data = lfilter(b, a, angle)
    return filtered_data
    

def server_init(receive_data, receive_data_event, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=64):
    s = socket.socket(type=SOCK_DGRAM)
    s.bind((HOST, PORT))

    print("Starting a server\n")
    print(str(HOST)+ ":" + str(PORT))
    
    receiving_data = True
    print("Started reading data\n")
   
    
    while receiving_data:
        data = s.recv(BUFF_SIZE)
        if data != b'Hello, UDP!':
            num_float = BUFF_SIZE//4
            received_floats = struct.unpack(f'{num_float}f', data)
            # print(received_floats)

            receive_data.put(list(received_floats))
            receive_data_event.set()
        
        
        
    s.close()
    print("Shutting Server Down ... \n")


def trajectory_motion(robot, motion_data_event, motion_data):
    while True:
        motion_data_event.wait()
        motion_data_event.clear()
        
        positions = []
        time.sleep(2)
        while not motion_data.empty():
            positions.append(motion_data.get())
        
        
        list_type = ["joint"] * len(positions)
        robot.execute_trajectory_from_poses_and_joints(positions, list_type, DIST_SMOOTHING)
        




def jog_motion(robot, receive_data_event, receive_data):
    robot.set_jog_control(True)
    receive_data_event.wait()
    receive_data_event.clear()
    # fil_b, fil_a = butter_lowpass(cutoff=1.25, fs=3, order=4)    
    while True:
      if not receive_data.empty():
        current = receive_data.get()
        break
    while True:
        if not receive_data.empty():
            next = receive_data.get()
            data = [a - b for a, b in zip(next, current)]
            # robot.jog_joints(data[1], data[0], data[4], data[3], data[6], data[8])
            # filtered_data = lfilter(fil_b, fil_a, data)
            # robot.jog_joints(round(filtered_data[8],1), round(filtered_data[6],1),round(filtered_data[3],1),round(filtered_data[4],1),round(filtered_data[0],1),round(filtered_data[1],1))
            robot.jog_joints(0,0,data[3],0,0,0)
            current = next


def main():
    try:  # Connect to robot and calibrate
        
        robot = NiryoRobot(WIFI_IP_ADDRESS)
        robot.calibrate_auto()
        robot.update_tool()
        
        receive_data = queue.Queue()
        receive_data_event = threading.Event()
        
        server_thread = threading.Thread(target=server_init, args=(receive_data, receive_data_event, HOST, PORT, BUFF_SIZE))
        server_thread.start()
                
        # different methodsof controling the robot
        # trajectory_motion(motion_data_event, motion_data)
        robot.jog_joints(0,0,0.5,0,0,0)
        jog_motion(robot, receive_data_event, receive_data)

        robot.go_to_sleep()
        robot.close_connection()
            
            
    except Exception as e:
        print(e)
        


if __name__ == '__main__':
    main()
  
  
