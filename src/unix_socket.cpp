#include "unix_socket.h"
#include <chrono>
#include <thread>

UnixSocketServer::UnixSocketServer() {
    std::cout << "Creating Unix socket for comms with python interface...\n";
    sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if(sock == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }
    memset(&server_addr, 0, sizeof(struct sockaddr_un));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, "/tmp/snr_socket.sock", sizeof(server_addr.sun_path) - 1);
    
    // Remove old socket file if it exists
    unlink(server_addr.sun_path);
    if(bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_un)) == -1) {
        perror("bind");
        exit(EXIT_FAILURE);
    }
}
void UnixSocketServer::sendMessage(const void *msg , size_t len) {
    int bytes_sent = sendto(sock, msg, len, 0, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_un));
    
    if(bytes_sent != len) {
        perror("sendto");
        exit(EXIT_FAILURE);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}


UnixSocketServer::~UnixSocketServer() {
    close(sock);
    unlink(server_addr.sun_path);
}
