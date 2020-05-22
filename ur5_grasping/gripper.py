#!/usr/bin/env python3

import sys
import threading
import socket
from threading import *


class GripperClient:
    """
    Interface for gripper
    """

    def __init__(self):
        # self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.host = '169.254.6.80'
        # self.port = 30003
        pass

    def open(self):
        """
        Command sucker to open
        """   
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('169.254.6.80', 30003))
        client.send(b"set_digital_out(0,True)\n")
        data  = client.recv(1024)
        client.close()
        print("gripper oFF")

    def close(self):
        """
        Command sucker to close
        """
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('169.254.6.80', 30003))
        client.send(b"set_digital_out(0,False)\n")
        data  = client.recv(1024)
        client.close()
        print("gripper ON")


if __name__ == "__main__":
    print("with argument \'close\' or \'open\'")
    gripper = GripperClient()
    if str(sys.argv[1]) == "open":
        gripper.open()
    elif str(sys.argv[1]) == 'close':
        gripper.close()
    else:
        print("Input invaild")
