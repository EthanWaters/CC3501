#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <poll.h>
#include <iostream>
#include <unistd.h>

//call it like ./client host port msg
void clearPreviousLine() {
    // Move the cursor to the beginning of the line and clear it
    std::cout << "\033[A\033[K";
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("argc less than 3");
        return 1;
    }
	
    /*
    Use getaddrinfo to generate an address structure corresponding to the host
    to connect to.
    */
    struct addrinfo hints;
    struct addrinfo *localAddress;  // For local address (for receiving)
    struct addrinfo *remoteAddress; // For remote address (for sending)

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_INET;      // IPv4
    hints.ai_socktype = SOCK_DGRAM; // UDP
    hints.ai_flags = AI_PASSIVE | AI_NUMERICHOST; // Interpret a NULL hostname as a wildcard (to accept data from anywhere)

    // Resolve the local address and port
    int s_local = getaddrinfo(nullptr, argv[2], &hints, &localAddress);
    if (s_local != 0) {
        fprintf(stderr, "Failed to resolve local address: %s\n", gai_strerror(s_local));
        return 1;
    }

    // Resolve the remote address and port
    int s_remote = getaddrinfo(argv[1], argv[2], &hints, &remoteAddress);
    if (s_remote != 0) {
        fprintf(stderr, "Failed to resolve remote address: %s\n", gai_strerror(s_remote));
        freeaddrinfo(localAddress);
        return 1;
    }

    // Open the socket for receiving (local address)
    int socket_fd = socket(localAddress->ai_family, localAddress->ai_socktype, localAddress->ai_protocol);
    if (socket_fd == -1) {
        perror("Failed to create socket");
        freeaddrinfo(localAddress);
        freeaddrinfo(remoteAddress);
        return 1;
    }

    // Bind the socket to the local address and port
    if (bind(socket_fd, localAddress->ai_addr, localAddress->ai_addrlen) != 0) {
        perror("Failed to bind");
        close(socket_fd);
        freeaddrinfo(localAddress);
        freeaddrinfo(remoteAddress);
        return 1;
    }
    
    

    // Allow multiple applications to use the same port (to run two versions of the app side by side for testing)
    int optval = true;
    if (0 != setsockopt(socket_fd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval))) {
        perror("Failed to set SO_REUSEPORT");
        return 1;
    }
    if (0 != setsockopt(socket_fd, SOL_SOCKET,  SO_BROADCAST, &optval, sizeof(optval))) {
        perror("Failed to set SO_BROADCAST");
        return 1;
    }

   

    // Prepare the pollfd array with the list of file handles to monitor
    struct pollfd pfds [] = {
        {
            // monitor the socket
            .fd = socket_fd,
            .events = POLLIN,
        },
        {
        // monitor stdin for user input
        .fd = STDIN_FILENO,
        .events = POLLIN,
		}
        // add here if there are other files/sockets to monitor
    };

    // Event loop
    char buf [32+1+240+1];
    char buffer_to_send [32+1+240+1];
    char buf_input [240];
    
    // initialise buffer empty
    memset(buffer_to_send, 0, sizeof(buffer_to_send));
	memset(buf, 0, sizeof(buf));
	memset(buf_input, 0, sizeof(buf_input));
    for (;;) {
        // Wait for events
        poll(pfds, sizeof(pfds)/sizeof(struct pollfd), -1);

        // Check if a packet arrived
        if (pfds[0].revents) {
            // Read the incoming packet
            ssize_t bytes_read = read(socket_fd, buf, sizeof(buf) - 2); // with room for a trailing null
            if (bytes_read < 0) {
				perror("Failed to recieve.");
				close(socket_fd);
                freeaddrinfo(localAddress);
                freeaddrinfo(remoteAddress);
                return 0;
            }
            // Make the message null terminated
            buf[bytes_read] = 0;
            buf[bytes_read+1] = 0;
            printf("%s\n", buf);
            memset(buf, 0, sizeof(buf));
        }
        if (pfds[1].revents) {
			// Send the message 
			fgets(buf_input, sizeof(buf_input), stdin);
			clearPreviousLine();
			buf_input[strcspn(buf_input, "\n")] = 0; // Remove newline
            if(buffer.find("\x03\x18") != std::string::npos){
                std::cout << "Program Terminated" << std::endl;
                close(socket_fd);
                freeaddrinfo(localAddress);
                freeaddrinfo(remoteAddress);
            }
            
			strcat(buffer_to_send, buf_input);
			strcat(buffer_to_send, "\0");
			s_remote = sendto(socket_fd, buffer_to_send, strlen(buffer_to_send), 0, remoteAddress->ai_addr, remoteAddress->ai_addrlen);
			if (s_remote == -1) {
				perror("Failed to send.");
				close(socket_fd);
                freeaddrinfo(localAddress);
                freeaddrinfo(remoteAddress);
				return 1;
			}
			memset(buffer_to_send, 0, sizeof(buffer_to_send));
			memset(buf_input, 0, sizeof(buf_input));
        }
    }
    
    // Free the memory returned by getaddrinfo
    freeaddrinfo(localAddress);
    freeaddrinfo(remoteAddress);
				

    // Close the socket
    close(socket_fd);
}
