#!/usr/bin/env python3
""" 
    Node listening to messages for the power monitoring node in order 
    to test it. Could just also just launch it and echo the ros topic. 

    This node is purely a testing tool. 
"""

import rospy

from rf_common.msg import RFPowerMonitorData

def pm_callback(data):
    rospy.loginfo("voltage: %f, current: %f, power: %f\r\n" % (data.battery_voltage, data.current, data.power))

def listen_for_power_messages():
    rospy.init_node('rf_power_monitor', anonymous=True)
    rospy.Subscriber("rf_power_monitor_topic", RFPowerMonitorData, pm_callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        listen_for_power_messages()
    except rospy.ROSInterruptException:
        pass
    



    
