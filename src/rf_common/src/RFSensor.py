#!/usr/bin/env python3
"""
    Base class for all the RF sensor types.
"""
import time
import subprocess

import rospy

from board import SCL, SDA
import busio

#---------------------------------------------------------------------
# RFSensor
#---------------------------------------------------------------------
class RFSensor:
    def __init__(self):
        """ """

    def update(self, data):
        """ """


