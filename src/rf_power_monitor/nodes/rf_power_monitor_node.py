#!/usr/bin/env python3
""" 
    power_monitor_node reads power information from a high side INA260 power 
    monitor. Power data is published to rf_power_monitor_topic
"""
import rospy
import time
import board
import adafruit_ina260

from rf_power_monitor.msg import RFPowerMonitorData

def publish_message(i2c, ina260):
    # topic, message type, queue size
    pub = rospy.Publisher('rf_power_monitor_topic',
                          RFPowerMonitorData, queue_size=10)

    rospy.init_node('rf_power_monitor_node', anonymous=True)
    #    rate = rospy.Rate(0.05) # 0.05 Hz is once ever 20 seconds
    rate = rospy.Rate(0.25) 

    # create custom message
    pwr_msg = RFPowerMonitorData()
    while not rospy.is_shutdown():
        pwr_msg.battery_voltage = ina260.voltage
        pwr_msg.power = ina260.power
        pwr_msg.current = ina260.current

        # debug_str = "Voltage: %s, Current: %s, Power: %s", pwr_msg.battery_voltage, pwr_msg.power, pwr_msg.current
        # rospy.loginfo(debug_str)

        pub.publish(pwr_msg)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        # To Do: Both of these next two lines need better error handling
        #        specifically if the address for the i2C line is other
        #        that the default the adafruit driver looses it. 
        i2c = board.I2C()
        ina260 = adafruit_ina260.INA260(i2c)

        publish_message(i2c, ina260)
    except rospy.ROSInterruptException:
        pass
    
