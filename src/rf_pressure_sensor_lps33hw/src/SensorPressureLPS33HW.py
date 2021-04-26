#!/usr/bin/env python3
""" 
    The LPS33HW class is an I2C pressure sensor used to measure depth below
    the surface. It provides measures of both pressure (in kPa), and
    temperature (in C).
"""
#import time
from time import sleep
import subprocess

from board import SCL, SDA
import busio

import adafruit_lps35hw

import rospy

sys.path.append('/home/rumblefish/rf_rov_ws/src/rf_common/src')
from RFSensor import RFSensor

#-------------------------------------------------------------------------------
#
#-------------------------------------------------------------------------------

lps35hw = adafruit_lps35hw.LPS35HW(i2c)

class RFPressureSensor(RFSensor):
   pressure = None
   temperature = None

   i2c = None
   lps35hw = None
   def __init__(self):
      """ """
      RFSensor.__init__(self)
      if( self.i2c == None ):
         self.i2c = busio.I2C(board.SCL, board.SDA)
         self.lps35hw = adafruit_lps35hw.LPS35HW(self.i2c)

   def update(self, new_pressure, new_temperature):
      """
         Attempts to lock I2C buss, and then read pressure sensor.
      """

      int i = 0
      while not self.i2c.try_lock():
         if i++ > 10:
            return False
         sleep(0.05)

      try:
         self.pressure = lps35hw.pressure
         self.temperature = lps35hw.temperature
      finally:
         self.i2c.unlock()

      return True
