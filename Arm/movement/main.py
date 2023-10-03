"""PI to ARM code. Ethan, Stuart & Lachlan"""

from pyniryo import *
from pynput import keyboard
import threading
import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
from typing import Any
import queue
import time

WIFI_IP_ADDRESS = "10.10.10.10"
DIST_SMOOTHING = 0.05
HOST = "127.0.0.1"
PORT = 54321
BUFF_SIZE=2048
SPEED = 500



def server_init(receive_data, receive_data_event, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=2048):
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
        receive_data.put(data)
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
    while not receive_data.empty():
        data = receive_data.get()
        data = eval(data)
        if isinstance(data, list):
            robot.jog_joints(data)
        elif data == 1:
            robot.close_gripper(SPEED)
        elif data == 2:
            robot.open_gripper(SPEED)
    
   
   
def command(robot, command_data_event, command_data):
    while True:
        command_data_event.wait()
        command_data_event.clear()
        while not command_data.empty():  
            command = command_data.get()
            if command == 1:
                robot.close_gripper(SPEED)
            elif command == 2:
                robot.open_gripper(SPEED)
    


def main():
    try:  # Connect to robot and calibrate
        
        robot = NiryoRobot(WIFI_IP_ADDRESS)
        robot.calibrate_auto()
        robot.update_tool()
        
        receive_data = queue.Queue()
        receive_data_event = threading.Event()
        
        #command_data = queue.Queue()
        #command_data_event = threading.Event()

        server_thread = threading.Thread(target=server_init, args=(receive_data, receive_data_event, HOST, PORT, BUFF_SIZE))
        server_thread.start()
        
        #command_thread = threading.Thread(target=command, args=(robot, command_data_event, command_data))
        #command_thread.start()
        
        # different methodsof controling the robot
        # trajectory_motion(motion_data_event, motion_data)
        jog_motion(robot, receive_data_event, receive_data)

        robot.go_to_sleep()
        robot.close_connection()
            
            
    except Exception as e:
        print(e)
        


if __name__ == '__main__':
    main()
  
  
