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
#include <thread>
#include <mutex>



template <typename T>

Client::Client(const char* ip, const char* port)
{   
    
    this-> _ip = ip
    this-> _port = port
    
    this-> _buf = buf
    this-> _buffer_to_send = buffer_to_send
    this-> _localAddress = localAddress
    this-> _remoteAddress = remoteAddress
    this-> _remoteAddress = socket_fd
    
    // Event loop
    char buf [256];
    char buffer_to_send [256];
    
    // initialise buffer empty
    memset(buffer_to_send, 0, sizeof(buffer_to_send));
	memset(buf, 0, sizeof(buf));
	memset(buf_input, 0, sizeof(buf_input));

    
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
    int s_local = getaddrinfo(nullptr, port, &hints, &localAddress);
    if (s_local != 0) {
        fprintf(stderr, "Failed to resolve local address: %s\n", gai_strerror(s_local));
        return 1;
    }

    // Resolve the remote address and port
    int s_remote = getaddrinfo(ip, port, &hints, &remoteAddress);
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

    
    
}

//call it like ./client host port msg
void Client::clearPreviousLine() {
    // Move the cursor to the beginning of the line and clear it
    std::cout << "\033[A\033[K";
}


bool Client::recieve(){
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
        if(buf.find("\x03\x18") != std::string::npos){
            std::cout << "Program Terminated" << std::endl;
            close(socket_fd);
            freeaddrinfo(localAddress);
            freeaddrinfo(remoteAddress);
        }
        buf[bytes_read] = 0;
        buf[bytes_read+1] = 0;
        printf("%s\n", buf);
        memset(buf, 0, sizeof(buf));
    }
    return 1
}


bool Client::send(){
    strcat(buffer_to_send, "\0");
    s_remote = sendto(socket_fd, buffer_to_send, strlen(buffer_to_send), 0, remoteAddress->ai_addr, remoteAddress->ai_addrlen);
    if (s_remote == -1) {
        perror("Failed to send.");
        close(socket_fd);
        freeaddrinfo(localAddress);
        freeaddrinfo(remoteAddress);
        return 0;
    }
    memset(buffer_to_send, 0, sizeof(buffer_to_send));
    memset(buf_input, 0, sizeof(buf_input));
    return 1
}

void Client::init_thread(T& input){
    sending_thread = std::thread(&Client::SendingThread<T>, this, std::ref(input)); 
}


void Client::SendingThread(T& input){
    while(!stop_thread){
        std::unique_lock<std::mutex> lock(coordinates_mutex)
        coordinates_cv.wait(lock, [&] {return stop_thread || input != current_coordinates; });
        if(!stop_thread){
            std::string data_to_send = input
            lock.unlock()
            send(input)
            lock.lock()
            current_coordinates = input
        }
    }
}


void Client::close_thread(){
    if(sending_thread.joinable()){
     stop_thread = true;
     coordinates_cv.notify_all()
     sending_thread.join()   
    }
}


Client::~client(){
    
    // Free the memory returned by getaddrinfo
    freeaddrinfo(localAddress);
    freeaddrinfo(remoteAddress);
            
    // Close the socket
    close(socket_fd);
}
