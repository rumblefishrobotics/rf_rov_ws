#!/usr/bin/env python3
""" 
    Node generating test messages for the thruster_control_node. 

    Node generates patterns showing turning on / off patterns of thrust 
    across all the longfish thrusters. 
"""

import rospy
from rf_thruster_control.msg import RFThrusterControlData

ESC_GPIO_PINS = [4,17,18,27]

def publish_message():
    # topic, message type, queue size
    pub = rospy.Publisher('rf_thruster_control_topic',
                          RFThrusterControlData, queue_size=10)

    rospy.init_node('rf_thruster_control_tester', anonymous=True)
    rate = rospy.Rate(0.2) # 0.2 hz

    # create custom message
    #
    ctrl_msg = RFThrusterControlData()
    ctrl_msg.thrusters_to_update = []
    for i in range(len(ESC_GPIO_PINS)):
        ctrl_msg.thrusters_to_update.append(ESC_GPIO_PINS[i])

    # Repeatedly cycle through thrusts...
    rate = rospy.Rate(0.2) # 0.2 hz
    cur_thrust = 0

    # Thrusts specified in %s
    while not rospy.is_shutdown():
        #        if cur_thrust >= 100:
        if cur_thrust >= 55:
            cur_thrust = 0
        ctrl_msg.thruster_drives = []
        for i in range(len(ESC_GPIO_PINS)):
            ctrl_msg.thruster_drives.append(cur_thrust)
        cur_thrust += 1
        debug_str = "Published thruster control message at:", rospy.get_time(), "cur_thrust: ", cur_thrust
        rospy.loginfo(debug_str)
        pub.publish(ctrl_msg)
        rate.sleep()

#-------------------------------------------------------------------------------
# Python Launch Singleton
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
    



    
