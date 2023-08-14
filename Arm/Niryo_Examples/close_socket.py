import psutil
import socket

def close_socket_by_address(address):
    # Get a list of all running processes
    for process in psutil.process_iter(['pid', 'name']):
        try:
            # Check if the process is a Python script
            if "python" in process.info['name'].lower():
                # Get the PID of the process
                pid = process.info['pid']
                
                # Get the open sockets used by the process
                connections = psutil.Process(pid).connections()
                
                # Iterate through the sockets and find the one with the desired address
                for conn in connections:
                    if conn.laddr and conn.laddr[0] == address[0] and conn.laddr[1] == address[1]:
                        # Close the socket
                        sock = socket.fromfd(conn.fd, socket.AF_INET, socket.SOCK_STREAM)
                        sock.close()
                        print(f"Closed socket with address {address}")
                        return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
    
    print(f"No socket found with address {address}")
    return False

if __name__ == "__main__":
    target_address = ("127.0.0.1", 65432)  # Replace this with the address you want to close
    close_socket_by_address(target_address)
