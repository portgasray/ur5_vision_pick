
from __future__ import print_function

import copy
import importlib
import os
import sys
import threading
import time
from abc import ABCMeta, abstractmethod

class Camera(object):
    """
    This is a parent class on which the robot
    specific Camera classes would be built.
    """
    __metaclass__ = ABCMeta

    def __init__(self, configs):
        """
        Constructor for Camera parent class.
        :param configs: configurations for camera
        :type configs: YACS CfgNode
        """
        self.configs = configs

    @abstractmethod
    def get_rgb(self, **kwargs):
        pass