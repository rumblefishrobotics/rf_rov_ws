#!/usr/bin/env python3
""" 
    The LPS33HW class is an I2C pressure sensor used to measure depth below
    the surface. It provides measures of both pressure (in kPa), and
    temperature (in C).
"""
import time
import subprocess

from board import SCL, SDA
import busio

import rospy
import adafruit_lps35hw

#-------------------------------------------------------------------------------
#
#-------------------------------------------------------------------------------


class Sensor:
   def __init__(self):
      """ """
      #...

   def update(self, foo):
      """ """
      #...



#-------------------------------------------------------------------------------
# 
#-------------------------------------------------------------------------------
#def shutdown_hook():
#    Cleanup code...

def pressure_callback(data):
   """Pressure sensor data recieved callback."""
   nodename = rospy.get_name()

    rospy.loginfo("(%s) heard: pres (%s) hPa, temp (%s) C,(%s),(%s)" % (rospy.get_name(),
                                                                        data.screen_line1,
                                                                        data.screen_line2))

def listen_for_messages():
    """Configure Subscriber"""
    #init_pressure_sensor()

    rospy.init_node("rf_pressure_node")
    #(topic),(custom message name),(name of callback function)
    rospy.Subscriber("rf_pressure_topic", RFPressureData, pressure_callback)
    rospy.spin()

    # Register the ROS shutdown hook
    # rospy.on_shutdown(shutdown_hook)


if __name__ == '__main__':
    listen_for_messages()

    
