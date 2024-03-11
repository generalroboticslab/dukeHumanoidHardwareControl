#include <iostream>
#include <msgpack.hpp>
#include <vector>
#include <string>
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <arpa/inet.h> 
#include <unistd.h> // For close()

#include "person.h" 

int main() {
    // Create a Person object
    // Person person{"Alice", 25, {"reading", "hiking", "coding"}};
    Person person{"Alice", 25, {1.1, -2.0, 3.0}};

    // Serialize using MsgPack
    msgpack::sbuffer buffer; 
    msgpack::pack(buffer, person);

    // Socket setup
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0); 
    if (sockfd < 0) {
        std::cerr << "Error creating socket\n";
        return 1;
    }

    // Destination setup (loopback for testing)
    sockaddr_in server_addr; 
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    // server_addr.sin_port = htons(12345); 
    // inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr); 

    server_addr.sin_port = htons(9870); 
    // inet_pton(AF_INET, "0.0.0.1", &server_addr.sin_addr); 
    inet_pton(AF_INET, "10.197.197.153", &server_addr.sin_addr); 

    // lab2: 10.194.20.206

    // Send data
    if (sendto(sockfd, buffer.data(), buffer.size(), 0,
           (const struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Error sending data\n";
        return 1;
    }

    close(sockfd); 
    return 0;
}
