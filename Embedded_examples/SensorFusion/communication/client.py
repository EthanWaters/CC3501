import socket

######################
####### CLIENT #######
###################### 

with socket.socket() as sock:
    print("Connecting...")
    sock.connect(('127.0.0.1', 12345))  # 192.168.0.5
    print("Connected")

    while True:
        send_string = input("S: ")
        sock.send(send_string.encode());

        if(send_string.lower() == "exit"):
            print("Ending session.")
            break

        # print("N:",sock.recv(1024).decode())
