#!/usr/bin/env python3
""" 
    Node generating test messages for the display node. Could just use
    rostopic pub to rf_display/rf_display_topic RFDisplayData <payload>,
    but payload is a fair bit to keep typing / editing. This node is
    purely a testing tool. 
"""

import rospy
#from std_msgs.msg import String
from rf_common_msgs.msg import RFDisplayData

from rf_common_msgs.msg import RFPowerMonitorData

def publish_rf_power_messages():
    # Push out test power data
    pub = rospy.Publisher('rf_power_monitor_topic',
                          RFPowerMonitorData, queue_size=10)

    rospy.init_node('rf_display_tester_node', anonymous=True)
    rate = rospy.Rate(0.2) # 0.2 hz

    # create custom message
    power_msg = RFPowerMonitorData()

    count = 0
    while not rospy.is_shutdown():
        count = count + 1

        # Count just adds some changing text, on each line, with each message
        power_msg.battery_voltage = count
        power_msg.current = count
        power_msg.power = count

        rospy.loginfo(power_msg)
        pub.publish(power_msg)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        publish_rf_power_messages()
    except rospy.ROSInterruptException:
        pass
    



    
