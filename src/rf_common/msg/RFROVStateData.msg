#--------------------------------------------------------------------------------
# Estimator listens to sensors, and thrust control messages, and creates an
# the RFRovState messages which contain the current estimate of the ROVs
# position, orientation, linear and angular velocity.
#--------------------------------------------------------------------------------
Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
float32 depth
