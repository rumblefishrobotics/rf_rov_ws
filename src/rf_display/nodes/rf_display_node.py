#!/usr/bin/env python3
""" 
    rf_display provides four lines of text, stacked one above the other. Each
    line has up to 32 characters. 
"""
import time
import subprocess

from board import SCL, SDA
import busio
import rospy
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

from rf_display.msg import RFDisplayData

#-------------------------------------------------------------------------------
# Constants
#-------------------------------------------------------------------------------

RF_DISPLAY_WIDTH = 128
RF_DISPLAY_HEIGHT = 32

#-------------------------------------------------------------------------------
# 
#-------------------------------------------------------------------------------
#def shutdown_hook():
#    global g_draw
#    # Clear the display
#    g_display.fill(0)
#    g_display.show()

def init_display():
   global g_i2c
   global g_display
   global g_draw
   global g_image
   global g_font
   
   g_i2c = busio.I2C(SCL, SDA)
   # Parameter 1: pixel width, 2: pixel heigh
   g_display = adafruit_ssd1306.SSD1306_I2C( RF_DISPLAY_WIDTH, RF_DISPLAY_HEIGHT, g_i2c)
   
   # Clear the display
   g_display.fill(0)
   g_display.show()

   # Create a blank image for drawing
   width = g_display.width 
   height = g_display.height
   g_image = Image.new("1", (width, height))

   # Draw a filled box to clear the image
   g_draw = ImageDraw.Draw(g_image)
   g_draw.rectangle((0,0, width, height), outline=0, fill=0)


   g_font = ImageFont.load_default()
   
def callback(data):
    nodename = rospy.get_name()

    rospy.loginfo("(%s) heard: (%s),(%s),(%s),(%s)" % (rospy.get_name(), data.screen_line1, data.screen_line2, data.screen_line3, data.screen_line4))

    g_draw.rectangle((0, 0, RF_DISPLAY_WIDTH, RF_DISPLAY_HEIGHT), outline=0, fill=0)
    top = -2


  
    g_draw.text((0, top +  0), data.screen_line1, font=g_font, fill=255)
    g_draw.text((0, top +  8), data.screen_line2, font=g_font, fill=255)
    g_draw.text((0, top + 16), data.screen_line3, font=g_font, fill=255)
    g_draw.text((0, top + 24), data.screen_line4, font=g_font, fill=255)
    
    g_display.image(g_image)
    g_display.show()
    
def listen_for_messages():
    """Configure Subscriber"""
    init_display()

    rospy.init_node("rf_display_node")
    #(topic),(custom message name),(name of callback function)
    rospy.Subscriber("rf_display_topic", RFDisplayData, callback)
    rospy.spin()

    # Register the ROS shutdown hook
    # rospy.on_shutdown(shutdown_hook)
    
if __name__ == '__main__':
    listen_for_messages()

    
