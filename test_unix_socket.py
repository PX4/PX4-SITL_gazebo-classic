# Used to test functionality of unix_socket.cpp class when integrated into mavlink_interface.cpp

import socket
import os
import asyncio


class testClass():
    def __init__(self):
        self.windowSize = 5
        self.window = []
        self.currSnrAverage = 0
        self.socket = self.formConnection()
        self.loop = asyncio.get_event_loop() 

    def formConnection(self):
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

        return sock

    async def readSocket(self):
        i = 0
        while True:
            # print("\nwaiting to receive message")
            data, address = self.socket.recvfrom(1024)
            data = data.decode('utf-8')
            data = data.rstrip('\x00')
            print("received {} bytes from {}.  Message # {}".format(len(data), address, i))
            print(data)
            if data != '':
                self.addToWindow(int(data))

            print("SNR Window AVG: ", self.currSnrAverage)
            i+=1
            await asyncio.sleep(0.001)

    def addToWindow(self, snr_val):
        if len(self.window) < self.windowSize:
            self.window.append(snr_val)
        else:
            self.currSnrAverage = sum(self.window)/self.windowSize
            self.window = []

    async def justATest(self):
        while True:
            print("Hello")
            await asyncio.sleep(0.001)

    def runner(self):
        asyncio.ensure_future(self.readSocket()) 
        asyncio.ensure_future(self.justATest())
        self.loop.run_forever()

def main():
    test = testClass()
    test.runner()

main()





