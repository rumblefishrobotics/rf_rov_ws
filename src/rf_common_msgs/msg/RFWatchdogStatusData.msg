#-------------------------------------------------------------------------------
#
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

