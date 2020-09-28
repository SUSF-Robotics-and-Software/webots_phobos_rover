# Simple test to see if an outside exec can connect to webots when a zmq socket
# is running

import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)

socket.connect("tcp://localhost:5000")

while True: 
    socket.send_string("Hello")

    time.sleep(0.001)

    print(socket.recv_string())