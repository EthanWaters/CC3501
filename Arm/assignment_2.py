"""PI to ARM code. Ethan, Stuart & Lachlan"""

from pyniryo import *
from pynput import keyboard
import threading
import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
#from Client import Client
#from Server import Server
from typing import Any
import queue
import time

WIFI_IP_ADDRESS = "10.10.10.10"
DIST_SMOOTHING = 0.05
HOST = "127.0.0.1"
PORT = 54321
BUFF_SIZE=1024



def server_init(received_data, event, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=1024):
    s = socket.socket(type=SOCK_DGRAM)
    s.bind((HOST, PORT))

    print("Starting a server\n")
    print(str(HOST)+ ":" + str(PORT))
    
    receiving_data = True
    print("Started reading data\n")
   
    
    while receiving_data:
        data = s.recv(BUFF_SIZE)
        data = data.decode('utf-8')
        if data == "end":
            receiving_data = False
            
        received_data.put(data)
        event.set()
    s.close()
    print("Shutting Server Down ... \n")


def trajectory_motion(robot, data_available_event, received_data):
    while True:
        data_available_event.wait()
        data_available_event.clear()
        
        positions = []
        
        time.sleep(2)
        while not received_data.empty():
            positions.append(received_data.get())
        
        
        list_type = ["joint"] * len(positions)
        positions = [eval(i) for i in positions]
        
        print(list_type)
        print(positions)
        
        robot.execute_trajectory_from_poses_and_joints(positions, list_type, DIST_SMOOTHING)
        

def jog_motion(robot, data_available_event, received_data):
    robot.set_jog_control(True)
    while True:
        data_available_event.wait()
        data_available_event.clear()
        
        if received_data.empty:
            time.sleep(0.2)
            
        move = received_data.get()
        move = eval(move)
        robot.jog_joints(move) 
            



def main():
    try:  # Connect to robot and calibrate
        
        robot = NiryoRobot(WIFI_IP_ADDRESS)
        robot.calibrate_auto()
        robot.update_tool()
        
        received_data = queue.Queue()
        data_available_event = threading.Event()

        processing_thread = threading.Thread(target=server_init, args=(received_data, data_available_event, HOST, PORT, BUFF_SIZE))
        processing_thread.start()
        
                                                   
        #server_init(received_data, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=1024)
        
        # different methodsof controling the robot
        # trajectory_motion(data_available_event, received_data)
        jog_motion(robot, data_available_event, received_data)

        robot.go_to_sleep()
        robot.close_connection()
            
            
    except Exception as e:
        print(e)
        


if __name__ == '__main__':
    main()
  
  
