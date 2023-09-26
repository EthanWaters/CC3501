#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <cstring.h>
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
		bool recieve();
		bool send(T& input);
		void init_thread(T& input);
		void sending_thread(T& input);
		void close_thread();
	
	private:
		struct addrinfo *localAddress;  // For local address (for receiving)
		struct addrinfo *remoteAddress; // For remote address (for sending)
		const char* ip;
		const char* port;
};
