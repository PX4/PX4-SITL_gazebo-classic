#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

class UnixSocketServer {
private:
    int sock;
    struct sockaddr_un server_addr;
public:
    UnixSocketServer();
    void sendMessage(const void *msg , size_t len);
    ~UnixSocketServer();
};
