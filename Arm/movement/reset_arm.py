from pyniryo import *

import threading
import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
from typing import Any
import queue
import time

WIFI_IP_ADDRESS = "192.168.0.177"


robot = NiryoRobot(WIFI_IP_ADDRESS)
robot.calibrate_auto()
robot.update_tool()


robot.go_to_sleep()
robot.close_connection()
