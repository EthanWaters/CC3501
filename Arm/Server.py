import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
from typing import Any
from Client import Client

class Server:
    def __init__(self, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=1024):
        self.s = socket.socket(type=SOCK_DGRAM)
        self.HOST = HOST
        self.PORT = PORT
        self.receiving_data = False
        self.received_data = []
        
        #self.MAX_CONNECTIONS = MAX_CONNECTIONS
        self.BUFF_SIZE = BUFF_SIZE
        self.s.bind((self.HOST, self.PORT))
        # self.s.listen(self.MAX_CONNECTIONS)
    
        print("Starting a server")
        print(str(self.HOST)+ ":" + str(self.PORT))
        
        # self.thread = threading.Thread(target=self.get_data)
        # self.thread.start()
        # self.s.setblocking(0)
        print(self.received_data)
        self.start_receiving()
    
    def __repr__(self):
        return "Server-{}:{} /n{}".format(self.HOST, self.PORT, self.received_data)  
        

    def start_receiving(self):
        self.receiving_data = True
        print("Started reading data")
    
    def stop_receiving(self):
        self.receiving_data = False
        self.s.close()
        print("Stopped reading data")


    def get_data(self):
        self.receiving_data = True
        self.received_data = []
        print(self.receiving_data)
        while self.receiving_data:
            data = self.s.recv(self.BUFF_SIZE)
            data = data.decode()
            print(data)
            print(self.received_data)
            if data == "end":
                self.stop_receiving()
            
            self.received_data = self.received_data.append(data) 
        print(self.received_data)

    def get_received_data(self):
        return self.received_data
    

    def set_received_data(self, value):
        self.received_data = value

server = Server()
#thread = threading.Thread(target=server.get_data())
#thread.start()  

