#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <cstring>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <poll.h>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <mutex>


class Client {
	public:
		Client(const char* ip, const char* port);
		void clearPreviousLine();
		int init();
		
		template <typename T>
		int send(T& input){
			const char* serialised_input = reinterpret_cast<const char*>(&input);
			size_t input_size = sizeof(input);
			s_remote = sendto(socket_fd, serialised_input, input_size, 0, remoteAddress->ai_addr, remoteAddress->ai_addrlen);
			if (s_remote == -1) {
				perror("Failed to send.");
				close(socket_fd);
				freeaddrinfo(localAddress);
				freeaddrinfo(remoteAddress);
				return 0;
			}
			return 1;
		}
				
		void close_thread();
	
	private:
		struct addrinfo *localAddress;  // For local address (for receiving)
		struct addrinfo *remoteAddress; // For remote address (for sending)
		struct addrinfo hints; 
		const char* _ip;
		const char* _port;
		int socket_fd;
		int s_remote;
		int s_local;
};
