# Used to test functionality of unix_socket.cpp class when integrated into mavlink_interface.cpp

import socket
import os

# the exact address of the socket
socket_address = '/tmp/snr_socket.sock'

# make sure the socket does not already exist
try:
    os.unlink(socket_address)
except OSError:
    if os.path.exists(socket_address):
        raise

# Create a UDS socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

# Bind the socket to the address
sock.bind(socket_address)

print("Starting up on {}".format(socket_address))
i = 0
while True:
    print("\nwaiting to receive message")
    data, address = sock.recvfrom(1024)

    print("received {} bytes from {}.  Message # {}".format(len(data), address, i))
    print(data)
    i+=1