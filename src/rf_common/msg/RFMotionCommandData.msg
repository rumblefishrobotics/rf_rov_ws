#-------------------------------------------------------------------------------
# Message encodes motion control commands - giving direction and thrust - which
# are used by the rf_motion_planner to generate thruster control commands.
#
# Version 1 is just a passthrough - taking motion commands such as forward or
# forward curve left and sending out Thruster control messages
#
#                             +-------------------+
#                             | rf_motion_planner |
#  -- RFMotionCommandData --> |                   |--> RFThrusterControlData
#                             |                   |
#                             +-------------------+
#  
#-------------------------------------------------------------------------------
Header header



float64 thrust_direction
int16[] thrust

# int16[] t


# Twist give velocity in linear and angular components



# forward
# back
#
# forward right
# back right
# forward left
# back left
#
# depth hold
#
# station hold

