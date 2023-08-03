class Client:
    
    def __init__(self, HOST="192.168.0.51", PORT=9077):
        self.s = socket.socket(type=SOCK_DGRAM)
        self.HOST = HOST
        self.PORT = PORT

    def send_message(self, message):
        self.s.sendto(str(message).encode(), (host, port))
