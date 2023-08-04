"""PI to ARM code. Ethan, Stuart & Lachlan"""

# from pyniryo import *
# from pynput import keyboard
import threading
import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
from Client import Client
from Server import Server

import time

WIFI_IP_ADDRESS = "10.10.10.10"
DIST_SMOOTHING = 0.1
HOST = "127.0.0.1"
PORT = 5432
BUFF_SIZE=1024


def listen_for_inputs(angle_inputs):
    while True:
        
        # CODE FOR GETTING DATA FROM EMBEDDED SYSTEM
        angles = get_angles_from_somewhere()
        if angles is not None:
            angle_inputs.append(angles)
        

def main():
    try:  # Connect to robot and calibrate
        #robot = NiryoRobot(WIFI_IP_ADDRESS)
        
        server = Server(HOST, PORT, BUFF_SIZE)
        #robot.calibrate_auto()
        #robot.update_tool()
        count = 0
        while count < 45:
            
            test2 = server.test
            if not test2.empty():
                print(test2.get())

            print(server.received_data)
            print("=======")
            count += 1
            time.sleep(1)
        print(count)
        server.stop_receiving()
        #angle_inputs = []
        #while True:
        #    angle_inputs = server.received_data
        #    print(angle_inputs)
        #    server.received_data = []
        
        #robot.go_to_sleep()
        #robot.close_connection()
    except Exception as e:
        print(e)
        server.stop_receiving()


if __name__ == '__main__':
    main()
  
  
