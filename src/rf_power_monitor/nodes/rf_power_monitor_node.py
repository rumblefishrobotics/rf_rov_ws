#!/usr/bin/env python3
""" 
    power_monitor_node reads power information from a high side INA260 power 
    monitor. Power data is published to rf_power_monitor_topic
"""
import rospy
import time
import board
import adafruit_ina260

from rf_common_msgs.msg import RFPowerMonitorData

import numpy as np
from scipy.ndimage.filters import uniform_filter1d

#-------------------------------------------------------------------------------
# Global Variables
#-------------------------------------------------------------------------------
g_sampling_rate = 1
g_publishing_rate = 20
g_pm_debugstr = False

g_voltages = []
g_currents = []
g_powers = []

#-------------------------------------------------------------------------------

def publish_message(i2c, ina260):
    """Samples power usage data, averages it, and then publishes it in an 
       infinite loop. """
    rate = rospy.Rate(g_sampling_rate) 

    # create custom message
    pwr_msg = RFPowerMonitorData()

    int i=0;
    while not rospy.is_shutdown():
        # Store the values in the ring buffer
        g_voltages[i] = ina260.voltage 
        g_currents[i] = ina260.current
        g_powers[i] = ina260.power
        
        pwr_msg.battery_voltage = np.average(g_voltages)
        pwr_msg.current = np.average(g_currents)
        pwr_msg.power = np.average(g_powers)
        
        i = i++
        if( i == g_publishing_rate):
            i = 0;
            pub.publish(pwr_msg)

            if(g_pm_debugstr):
                debug_str = "BV: %s, I: %s, P: %s" % (pwr_msg.battery_voltage,
                                                      pwr_msg.power,
                                                      pwr_msg.current)
                rospy.loginfo(debug_str)
            
        rate.sleep()

#-------------------------------------------------------------------------------

def load_config_params():
    """Loads params from YAML config file. """
    global g_sampling_rate
    global g_publishing_rate
    
    if (not rospy.has_param('pm_sampling_rate')) or \
       (not rospy.has_param('pm_publishing_rate')) or \
       (not rospy.has_param('g_pm_debugstr')):
        rospy.loginfo("rf_power_monitor unable to find config params. Using default.")
        g_sampling_rate = 0.2
        g_publishing_rate = 0.2
    else:
        g_sampling_rate = rospy.get_param('pm_sampling_rate')
        g_publishing_rate = rospy.has_param('pm_publishing_rate')        
        g_pm_debugstr = rospy.has_param('g_pm_debugstr')

        g_voltages = [0.0] * g_publishing_rate
        g_currents = [0.0] * g_publishing_rate
        g_powers   = [0.0] * g_publishing_rate

#-------------------------------------------------------------------------------

if __name__ == '__main__':
    load_config_params()
    try:
        # To Do: Both of these next two lines need better error handling
        #        specifically if the address for the i2C line is other
        #        that the default the adafruit driver looses it. 
        i2c = board.I2C()
        ina260 = adafruit_ina260.INA260(i2c)

        # topic, message type, queue size
        pub = rospy.Publisher('rf_power_monitor_topic',
                              RFPowerMonitorData, queue_size=10)

        rospy.init_node('rf_power_monitor_node', anonymous=True)
        #    rate = rospy.Rate(0.05) # 0.05 Hz is once ever 20 seconds
        
        # Warning: Publish_message is an infinite loop
        publish_message(i2c, ina260)
    except rospy.ROSInterruptException:
        pass
    
