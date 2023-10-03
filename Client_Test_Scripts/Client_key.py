import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
import time
from pynput import keyboard
from Client import Client

SHIFT = 0.05


def on_press(key):
    global current_key
    try:
        if key == keyboard.Key.shift:
            print('shift key!')
            current_key = ''
        else:
            current_key = key.char
    except AttributeError:
        print('special key {0} pressed'.format(
            key))
        current_key = ''


def on_release(key):
    global current_key
    current_key = ''
    if key == keyboard.Key.esc:
        return False
    else:
        current_key = ''


client = Client()
current_key = ''
listener = keyboard.Listener(on_press=on_press,on_release=on_release)
listener.start()


while listener.is_alive():
     while current_key != '':
        if current_key == '0':
            client.send_message("end")
        elif current_key == 'a':
            client.send_message([SHIFT, 0.0, 0.0, 0.0, 0.0, 0.0])
        elif current_key == 'd':    
            client.send_message([-SHIFT, 0.0, 0.0, 0.0, 0.0, 0.0])
        elif current_key == 's':
            client.send_message([0.0, SHIFT, 0.0, 0.0, 0.0, 0.0])
        elif current_key == 'w':    
            client.send_message([0.0, -SHIFT, 0.0, 0.0, 0.0, 0.0])
        elif current_key == 'q':
            client.send_message([0.0, 0.0, SHIFT, 0.0, 0.0, 0.0])
        elif current_key == 'e':    
            client.send_message([0.0, 0.0, -SHIFT, 0.0, 0.0, 0.0])
        elif current_key == '1':    
            client.send_message(1)
        elif current_key == '2':    
            client.send_message(2)
        time.sleep(0.05)
