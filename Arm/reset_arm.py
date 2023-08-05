from pyniryo import *
from pynput import keyboard
import threading
import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
from typing import Any
import queue
import time

WIFI_IP_ADDRESS = "10.10.10.10"


robot = NiryoRobot(WIFI_IP_ADDRESS)
robot.calibrate_auto()
robot.update_tool()


robot.go_to_sleep()
robot.close_connection()
