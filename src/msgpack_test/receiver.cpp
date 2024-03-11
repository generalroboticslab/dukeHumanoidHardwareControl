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
    // Socket setup
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0); 
    if (sockfd < 0) {
        std::cerr << "Error creating socket\n";
        return 1;
    }

    // Bind to port
    sockaddr_in my_addr; 
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(9870); 
    my_addr.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces

    if (bind(sockfd, (const struct sockaddr*) &my_addr, sizeof(my_addr)) < 0) {
        std::cerr << "Error binding socket\n";
        return 1;
    }

    // Receive data
    char buffer[1024]; 
    int bytes_received = recvfrom(sockfd, buffer, sizeof(buffer), 0, NULL, NULL);
    if (bytes_received < 0) {
        std::cerr << "Error receiving data\n";
        return 1;
    }

    // Deserialize
    msgpack::object_handle oh = msgpack::unpack(buffer, bytes_received);
    msgpack::object obj = oh.get();
    Person received_person;
    obj.convert(received_person);

    // Print received data
    std::cout << "Received:\n"
              << "Name: " << received_person.name << "\n"
              << "Age: " << received_person.age << "\n"
              << "Hobbies: ";
    for (const auto& hobby : received_person.hobbies) {
        std::cout << hobby << " ";
    }
    std::cout << std::endl;

    close(sockfd);
    return 0;
}
