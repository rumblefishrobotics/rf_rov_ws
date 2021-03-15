#!/usr/bin/env python3
""" 
    rf_display provides four lines of text, stacked one above the other. Each
    line has up to 32 characters. 
"""

import rospy
#from std_msgs.msg import String
from rf_display.msg import RFDisplayData

def callback(data):
    rospy.loginfo("%s: heard (%s),(%s)", rospy.get_name(), data.dbs_line1, data.dbs_line2)
    # Working debugging version just using a string. 
    # rospy.loginfo("%s: heard (%s)", rospy.get_name(), data)


def listen_for_messages():
    """Configure Subscriber"""
    #(topic),(custom message name),(name of callback function)
    rospy.Subscriber("rf_display_topic", RFDisplayData, callback)

    # Working debugging version just using a string.
    # rospy.Subscriber("debug_display_topic", String, callback)
    
if __name__ == '__main__':
    rospy.init_node("rf_display_node")
    listen_for_messages()
    rospy.spin()
    
