#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cstring>
#include <locale>
#include <codecvt>
#include <string>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <poll.h>
#include <iostream>
#include <unistd.h>


class Client {
	public:
		Client(const char* ip, const char* port);
		void clearPreviousLine();
		int init();
		int send(std::string input);
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
