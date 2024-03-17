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

void UnixSocketServer::sendMessage(uint8_t message) {
    // Convert uint8_t to void *
    void *msg = static_cast<void*>(&message);
    size_t len = sizeof(message); 

    // Original if condition and sending the message
    if(msg != NULL)
    {
        int bytes_sent = sendto(sock, msg, len, 0, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_un));
        if(bytes_sent != len) {
            perror("sendto");
            exit(EXIT_FAILURE);
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

uint8_t UnixSocketServer::AvgSnrArray(const void *snr, size_t len, uint8_t satellites_visbile){
    const uint8_t* snr_data = static_cast<const uint8_t*>(snr);
    uint16_t sum_ = 0;
    uint16_t numVals = 0;
    uint16_t avg = 0;

    // Use list of satellites in use to determine the values of the satellites_visible the we should use.


    if(satellites_visbile == 0) {
        return 0;
    }
    else {
        for(size_t i = 0; i < len; ++i) { 
            uint8_t currVal = snr_data[i];

            if (currVal != 0) {
                sum_ += currVal;
                ++numVals;
            }
        }

        if ( (sum_ != 0) && (numVals != 0) ) {
            
            avg = sum_ / numVals;
        }
    }

    return avg;
}


// Streamlined version of the above method, this assumes that the satellites_visible value is always correct
// uint8_t UnixSocketServer::AvgSnrArray(const uint8_t *snr, const uint8_t satellites_visbile){
//     if(satellites_visbile == 0) {
//         return 0;
//     }

//     size_t sum = 0;
//     for(size_t i = 0; i < satellites_visbile; ++i) {                                                                                                                                                                 
//         sum += snr[i]
//     }

//     return sum / satellites_visbile;
// }


UnixSocketServer::~UnixSocketServer() {
    close(sock);
    unlink(server_addr.sun_path);
}
