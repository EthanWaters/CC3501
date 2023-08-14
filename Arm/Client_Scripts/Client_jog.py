import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
import time
from pynput import keyboard
from Client import Client

SHIFT = 0.05
client = Client()

while listener.is_alive():
    for i in range(10):
        client.send_message([SHIFT, SHIFT, SHIFT, 0.0, 0.0, 0.0])
        time.sleep(0.05)
    for i in range(10):
        client.send_message([-SHIFT, -SHIFT, -SHIFT, 0.0, 0.0, 0.0])
        time.sleep(0.05)
