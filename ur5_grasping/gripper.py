#!/usr/bin/env python3

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

    def close(self):
        """
        Command sucker to close
        """
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('169.254.6.80', 30003))
        client.send(b"set_digital_out(0,False)\n")
        data  = client.recv(1024)
        client.close()


if __name__ == "__main__":
    gripper = GripperClient()
    gripper.close()
