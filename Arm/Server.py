import socket
from socket import SOCK_DGRAM, SO_REUSEADDR
import threading
from Client import Client

class Server:
    def __init__(self, HOST="127.0.0.1", PORT=54321, BUFF_SIZE=1024):
        self.s = socket.socket(type=SOCK_DGRAM)
        self.HOST = HOST
        self.PORT = PORT
        self.recieving_data = False
        self.recieved_data = []
        
        #self.MAX_CONNECTIONS = MAX_CONNECTIONS
        self.BUFF_SIZE = BUFF_SIZE
        self.s.bind((self.HOST, self.PORT))
        # self.s.listen(self.MAX_CONNECTIONS)
    
        print("Starting a server")
        print(str(self.HOST)+ ":" + str(self.PORT))
        
        self.thread = threading.Thread(target=self.get_data)
        self.thread.start()
        # self.s.setblocking(0)
        self.start_recieving()
    
    def __repr__(self):
        return "Server-{}:{} /n{}".format(self.HOST, self.PORT, self.recieved_data)  
        

    def start_recieving(self):
        self.recieving_data = True
        self.thread = threading.Thread(target=self.get_data)
        self.thread.start()
        print("Started reading data")
    
    def stop_recieving(self):
        self.recieving_data = False
        self.s.close()
        print("Stopped reading data")


    def get_data(self):
        self.recieving_data = True
        while self.recieving_data:
            data_recieved = self.s.recv(self.BUFF_SIZE)
            data = data_recieved.decode()           
            if data == "end":
                self.stop_recieving()
            self.recieved_data.append(data)
            
    def get_recieved_data(self):
        print(self.recieved_data)
        return self.recieved_data
    
server = Server()
