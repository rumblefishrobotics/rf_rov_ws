#-------------------------------------------------------------------------------
# Message encodes thruster the thruster control message, which encodes the drive 
# signals to be output to each of the the ESCs driving thruster motors. 
#
#                                 +---------------------+
#                                 | rf_thruster_control |--> Thruster Signal 1
#  -- ThrusterControl message --> |                     |         . . .
#                                 |                     |--> Thruster SIgnal N
#                                 +---------------------+
#
# The rf_thruster_control node is an abstraction. The current implementation 
# directly drives GPIO pines to PWM using the pigpiod interface. However it 
# deliberatly separates out PWM generation so that later a hardware I2C controlled
# signal generator, like the PCA9685, could be swapped in to unload the processor.
# This swap will likely be needed for platforms that move from 3 thrusters to
# designs using 8-12 thrusters. 
#
# ESC PWM Control Timing
# ----------------------
# Each thruser output is a digital PWM signal meant to drive a down stream ESC.
# Controllers are assumed to be running a BLHeli firmware.
#
# Standard PWM ESC Control Signal, output to thrusters 1..N. The ESC PWM control
# uses a (1000us-2000us) high pulse to encode motor speed. 
#
#   +--+ <- 20ms -> +-+                    +----+          +----+
#   |  |            | |                    |    |          |    |
#   |  |            | |                    |    |          |    |
# --+  +------------+ +-----             --+    +----------+    +----
# ->    <-- 1ms pulse width, Min RPM     -->    <-- 2ms pulse width, Max RPM
#
# While the signal control range is between 1000 and 2000 us, the usable control
# range is from 1100 to 1900 us. The center point of 1500 uS acts as the neutral
# position. A signal of 1900 us represent max forward, and a signal of 1100
# represents max reverse. There is a +/-25uS dead band arround the neutral center
# position.
#
# While Dshot (which is a digital protocol) has a resolution of 2048 steps, the PWMs
# analog signal has 1000 of accuracy. 
# 
# ESC Arming Command
# ------------------
# ESCs have a safety interlock and wont start unless there is an initial no
# movement setting during startup. This STOP signal is at 1500us, or neutral,
# and should last several seconds.
#
# Longfish uses an 8 second startup delay to ensure the system has powered
# up all the ESCs have had time to recognize the arming STOP command. 
#
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# ROS Thruster Data
#
# thruser-drive encodes the thrust levels for each thruster. The thruser_control
# node must have been previously configured with a ThrusterConfigData message
# sets the number of thrusters, their GPIO pin numbers, and the full scale range
# of thruser values.
#
# So if a thruser-drive value of 100 is sent on a thruster with a configured
# full scale range of 1000 the thruster is driven at 10%. If the same signal
# is sent to a thruster with a full scale range of 200 the thruster is driven
# at 50%. All thrusters must use the same full scale range. 
#-------------------------------------------------------------------------------
int16[] thrusters_to_update
int16[] thruster_drives

