import socket

######################
####### SERVER #######
###################### 

with socket.socket() as sock:
    port = 12345
    sock.bind(('', port))

    print("Socket starting, waiting for connection...")

    sock.listen(5)
    conn, addr = sock.accept()

    print("Socket Up and running with a connection from", addr)

    while True:
        received_data = conn.recv(1024).decode()

        print("S:", received_data)
        
        # sendData = input("N: ")
        # c.send(sendData.encode())

        if(received_data.lower() == "exit"):
            print("Session ended by client.")
            break
    