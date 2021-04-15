#-------------------------------------------------------------------------------
# System uses a INA260 power / current / voltage monitor to measure battery
# voltage (Vb), battey current (Vi), and .... regulated power.  
#
# Battery          +---------------------+    +--------+        +------------+
# Voltage - (Vi) ->| INA260              |--->| 5V reg |- 5V -->| Ras Pi and |
#  (Vb)            |                     |    +---+----+        | other HW   |
#                  +----------+----------+        |             +------+-----+
#                             |                   |                    |
#       GND ------------------+-------------------+--------------------+
# 
# If using Adafruit breakout board for the INA260 their tutorial is located here
# https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout
# 
#-------------------------------------------------------------------------------

float32 battery_voltage

# current and power are being measured for all the systems other than the motors
# which are powered directly from the battery.
float32 current
float32 power

