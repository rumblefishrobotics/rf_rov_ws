#!/usr/bin/env python3
"""
    rf_debug_display_node wraps the display generic rf_common_ws/rf_display_ssd1306
    display node. This node subsribes to various system messages, processes the 
    incomming data, and pushes it out to the wraped display. 
"""
import time
import subprocess

import rospy

from rf_common.msg import RFPowerMonitorData
from rf_common.msg import RFThrusterControlData
from rf_common.msg import RFWatchdogStatusData

from DisplaySSD1306 import DisplayLine
from DisplaySSD1306 import DisplaySSD1306

#-------------------------------------------------------------------------------
# Formatting constants
#-------------------------------------------------------------------------------
POWER_LINE = 1

#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

def power_monitor_callback(dp_data):
    """ """
    global dbg_display
    rospy.loginfo("power monitor callback on debug display")

    newline = "V%4.2f, I%5.1f, P%5.2f" % (dp_data.battery_voltage, dp_data.current, dp_data.power)
    dbg_display.update_display_line(POWER_LINE, newline)
    dbg_display.refresh_display()

def listen_for_messages():
    """Creates debug display node and starts listening for system data to push 
       to the underlying rf_debug_display_ssd1306 display."""
    rospy.init_node("rf_debug_display_node")

    rospy.Subscriber("rf_power_monitor_topic",
                     RFPowerMonitorData,
                     power_monitor_callback)
    rospy.spin()

#-------------------------------------------------------------------------------
# Startup source singleton
#-------------------------------------------------------------------------------

dbg_display = None
def init_display():
    """ """
    global dbg_display
    dbg_display = DisplaySSD1306()


if __name__ == '__main__':
    init_display()
    try:
        listen_for_messages()
    except rospy.ROSInterruptException:
        pass
