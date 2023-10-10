#include <Client.h>

Client::Client(const char* ip, const char* port)
{   
    
    this-> _ip = ip;
    this-> _port = port;
    

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
    
    int s_remote;
    int socket_fd = socket(localAddress->ai_family, localAddress->ai_socktype, localAddress->ai_protocol);
}


int Client::init()
{   
    // Resolve the local address and port
    int s_local = getaddrinfo(nullptr, _port, &hints, &localAddress);
    if (s_local != 0) {
        fprintf(stderr, "Failed to resolve local address: %s\n", gai_strerror(s_local));
        return 1;
    }

    // Resolve the remote address and port
    s_remote = getaddrinfo(_ip, _port, &hints, &remoteAddress);
    if (s_remote != 0) {
        fprintf(stderr, "Failed to resolve remote address: %s\n", gai_strerror(s_remote));
        freeaddrinfo(localAddress);
        freeaddrinfo(remoteAddress);
        return 1;
    }

    // Open the socket for receiving (local address)
    
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
    return 0;
}


void Client::clearPreviousLine() {
    // Move the cursor to the beginning of the line and clear it
    std::cout << "\033[A\033[K";
}


template <typename T>
int Client::send(T& input){
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

