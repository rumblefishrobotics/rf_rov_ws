#!/usr/bin/env python3
""" 
    Node publishes readings from the externally ported LPS33HW pressure sensor. 
"""
import time
import subprocess

from board import SCL, SDA
import busio

import rospy

#-------------------------------------------------------------------------------
# Global Variables
#-------------------------------------------------------------------------------
g_pressure_sampling_rate = 1

#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------
#def shutdown_hook():
#    Cleanup code...

def pressure_callback(data):
   """Pressure sensor data recieved callback."""
   nodename = rospy.get_name()

    rospy.loginfo("(%s) heard: pres (%s) hPa, temp (%s) C,(%s),(%s)" % (rospy.get_name(),
                                                                        data.screen_line1,
                                                                        data.screen_line2))


def monitor_pressure():
    """Creates a pressure sensor node, then starts regularly publishing measured
       pressure and temperature measurements."""
    global pressure_sensor
    rate = rospy.Rate(g_pressure_sampling_rate)

    pressure_msg = RFPressureData()

    while not rospy.is_shutdown():
      if not pressure_sensor.update():
         rospy.loginfo("(%s) Read of pressure sensor failed.")
      else:
         rospy.init_node("rf_pressure_node")
         rospy.Subscriber("rf_pressure_topic", 
                          RFPressureData,      
                          pressure_callback)   
      rospy.spin()

    # Register the ROS shutdown hook
    # rospy.on_shutdown(shutdown_hook)

#----------------------------------------------------------------------
# Startup source singleton
#----------------------------------------------------------------------
def load_config_params():
   """Loads params from YAML config file."""
   global g_pressure_sampling_rate

   if (not rospy.has_param('g_pressure_sampling_rate')):
      rospy.loginfo("rf_pressure_monitor unable to find config params. Using defaults.")
      g_pressure_sampling_rate = 1
   else:
      g_pressure_sampling_rate = rospy.get_param('pm_pressure_sampling_rate')

#----------------------------------------------------------------------
# Init
#----------------------------------------------------------------------
pressure_sensor = None
def init_sensor():
   """ """
   global pressure_sensor
   pressure_sensor = RFPressureSensor()

#----------------------------------------------------------------------
# Startup Singleton
#----------------------------------------------------------------------
if __name__ == '__main__':
   init_sensor()
   monitor_pressure()

    
