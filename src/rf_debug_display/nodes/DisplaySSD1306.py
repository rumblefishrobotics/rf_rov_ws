#!/usr/bin/env python3
"""
    The ssd1306 is a OLED display. This calss provides a wrapper for the
    Adafruit drivers for the display. It provides four lines of text,
    stacked one above the other. Each line has up to 32 characters.
"""
import time
import subprocess

import rospy

from PIL import Image, ImageDraw, ImageFont

from board import SCL, SDA
import busio

import adafruit_ssd1306

#-------------------------------------------------------------------------------
# Display Line
#-------------------------------------------------------------------------------

class DisplayLine:
    def __init__(self):
        """ """
        self.data = ""
        self.last_updated = 0

    def update(self, newline):
        """ """
        self.data = newline
        self.last_updated = time.localtime()

#-------------------------------------------------------------------------------
# Display Class
#-------------------------------------------------------------------------------
class DisplaySSD1306:
    DISPLAY_WIDTH = 128
    DISPLAY_HEIGHT = 32

    def __init__(self):
        """Startup initilization of display."""
        # Screen Buffer
        self.screen_line1 = DisplayLine();
        self.screen_line2 = DisplayLine();
        self.screen_line3 = DisplayLine();
        self.screen_line4 = DisplayLine();

        self.i2c = busio.I2C(SCL, SDA)
        self.display = adafruit_ssd1306.SSD1306_I2C(self.DISPLAY_WIDTH,
                                                    self.DISPLAY_HEIGHT, self.i2c)
        # Clear the display
        self.display.fill(0)
        self.display.show()

        # Create a blank image for drawing
        width = self.display.width
        height = self.display.height
        self.image = Image.new("1", (width, height))

        # Draw a filled box to clear the image
        self.draw = ImageDraw.Draw(self.image)
        self.draw.rectangle((0, 0, width, height), outline=0, fill=0)

        self.font = ImageFont.load_default()

#    def __del__(self):
#        """Clear the display on shutdown."""
#        # Clear the display
#        self.display.fill(0)
#        self.display.show()
        
    def update_display_line(self, line_number, line_data):
        """Pushes incomming data to display backing buffer, but does not 
           refresh the display."""

        if(not isinstance(line_data, str)):
            # Invalid data
            raise ValueError("Invalid data passed to update_display.")
            return
        
        if(line_number == 0):
            self.screen_line1.update(line_data)
        elif(line_number == 1):
            self.screen_line2.update(line_data)
        elif(line_number == 2):
            self.screen_line3.update(line_data)
        elif(line_number == 3):
            self.screen_line4.update(line_data)
        else:
            # Invlid line number
            raise ValueError("Invalid line number passed to display data.")

    def update_display(self,display_data):
        """Pushes incomming data to display backing buffer, but does not 
           refresh the display."""

        if((not isinstance(display_data,list)) or (len(display_data) != 4)):
           raise ValueError("Invalid data list passed to update_display.")

        self.screen_line1.update(display_data[0])
        self.screen_line2.update(display_data[1])
        self.screen_line3.update(display_data[2])
        self.screen_line4.update(display_data[3])           
        
    def refresh_display(self):
        """Pushes current screen buffer onto display."""

        self.draw.rectangle((0, 0, self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT),
                            outline=0, fill=0)
        top = -2

        self.draw.text((0, top +  0), self.screen_line1.data, font=self.font, fill=255)
        self.draw.text((0, top +  8), self.screen_line2.data, font=self.font, fill=255)
        self.draw.text((0, top + 16), self.screen_line3.data, font=self.font, fill=255)
        self.draw.text((0, top + 24), self.screen_line4.data, font=self.font, fill=255)

        self.display.image(self.image)
        self.display.show()
