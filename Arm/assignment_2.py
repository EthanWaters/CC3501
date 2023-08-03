"""PI to ARM code. Ethan, Stuart & Lachlan"""

from pyniryo import *
from pynput import keyboard

WIFI_IP_ADDRESS = "10.10.10.10"
DIST_SMOOTHING = 0.1




def main():
     # Connect to robot and calibrate
    robot = NiryoRobot(WIFI_IP_ADDRESS)
    robot.calibrate_auto()
    robot.update_tool()
    
    
    
    robot.go_to_sleep()
    robot.close_connection()



if __name__ == '__main__':
   
   main()
   
  