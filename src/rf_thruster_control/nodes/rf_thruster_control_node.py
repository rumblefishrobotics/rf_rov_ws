#!/usr/bin/env python3
import pigpio
import rospy
import time

from rf_thruster_control.msg import RFThrusterControlData

# Resources
# http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks

#-------------------------------------------------------------------------------
# Control Constants
#-------------------------------------------------------------------------------
ESC_GPIO_PINS = [4,17,18,27]   

# Signal frequency in HZ ((2000us * 50) = 1S)
ESC_PWM_FREQ = 50

# Number of steps in PWM control
ESC_PWM_RANGE = 1000

#-------------------------------------------------------------------------------
# Thruser Control
#-------------------------------------------------------------------------------
class ThrusterControl:
   """
       ROS class to contol PWM signals sent to the ESC board(s).
   """
   #Array of pin numbers array('B', [])

   def __init__(self, gpio_pin_nos, esc_pwm_freq, esc_pwm_range):
      """
      Initialises 
      """    
      self.esc_pwm_freq = esc_pwm_freq
      self.esc_pwm_range = esc_pwm_range  
      self.gpio_pin_nos = gpio_pin_nos

      self.pi = pigpio.pi()
      
      # Sets neutral center postion. All good ESCs have an auto-break feature where
      # the neutral drive signal *must* be detected on starup, otherwise they lock
      # up. 
      pwm_neutral = (self.esc_pwm_range / 2)

      # pdrives stores the drive % value for each ESC pin. To start
      # the ESCs need to see a "locked" signal with the control line
      # signalling 50% or idle. 
      self.pdrives = [pwm_neutral] * len(ESC_GPIO_PINS)

      for cur_pin_no in self.gpio_pin_nos:
         self.pi.set_PWM_frequency( cur_pin_no, self.esc_pwm_freq )
         self.pi.set_PWM_range( cur_pin_no, self.esc_pwm_range )
         self.pi.set_PWM_dutycycle( cur_pin_no, pwm_neutral )

      # Generate lock signal long enough for ESCs to detect it 
      time.sleep(1)
      return
   
   def update_ESCs(self, ESCs_to_update, drive_updates):
      if len(ESCs_to_update) != len(drive_updates):
         return False
      
      # Perform basic sanity checks
      for i in range(len(ESCs_to_update)):
         cur_pin_no = ESCs_to_update[i]
         cur_pdrive = drive_updates[i]
         if( cur_pin_no not in ESC_GPIO_PINS ):
            return False
         if( cur_pdrive < 0 or cur_pdrive > self.esc_pwm_range ):
            return False
         
      for i in range(len(ESCs_to_update)):
         cur_pin_no = ESCs_to_update[i]
         cur_pdrive = drive_updates[i]
         log_str = "\t\tupdate_ESCs tsetting GPIO (%s) to new drive (%s)"
         rospy.loginfo(log_str, cur_pin_no, cur_pdrive)
         self.pi.set_PWM_dutycycle(cur_pin_no, cur_pdrive)

      return True
      

   def ESCs_to_neutral(self):
      """
          Places all ESCs to neutral / locked position to stop the motors. 
      """
      for cur_pin_no in ESC_GPIO_PINS:
         range = self.pi.get_PWM_range( cur_pin_no )
         self.pi.set_PWM_dutycycle( cur_pin_no, (range * 0.5) )

      
   def thruster_error(self, terminal):
      """
          Called on any error configuring thrusters after a sucessfull init. 
          
          Error puts the ESCs at 50% duty cycle oscillation as that is the idle/
          stop/lock position for analog ESCs. 

          For terminal errors stop and restart pigpio services and then shut
          everything down. 
      """
      if terminal == True:
         self.pi.stop();
         self.pi = pigpio.pi()
         # make sure control constants are set to sane values
         self.esc_pwm_freq = ESC_PWM_FREQ
         self.esc_pwm_range = ESC_PWM_RANGE
         for i in range(len(ESC_GPIO_PINS)):
            self.pi.set_PWM_frequency(ESC_GPIO_PINS[i],
                                       self.esc_pwm_freq )
            self.pi.set_PWM_range(ESC_GPIO_PINS[i],
                                  self.esc_pwm_range )
         
      for cur_pin_no in ESC_GPIO_PINS:
         range = self.pi.get_PWM_range( cur_pin_no )
         pi.set_PWM_dutycycle( cur_pin_no, (range * 0.5) )

      if terminal == True:
         self.pi.stop();

#-------------------------------------------------------------------------------
# End of ThrusterControl Class Definition
#-------------------------------------------------------------------------------

def thruster_control_callback(cntrl_msg):
   """
       Callback message is a list of thrusters specified as Broadcom GPIO pin 
       numbers, and thrust at which to drive them. Thrusts specified as a positive
       integer in the drive range. 
   """
   global g_tcntrl
   if g_tcntrl == None:
       rospy.logerr("RF_ERROR: Callback with unset thrust control object!")
       return
   
   if(len(cntrl_msg.thrusters_to_update) != len(cntrl_msg.thruster_drives)):
       rospy.logerr("RF_ERROR: Bad thruser control message, (thrusters != thrusts)")
       tcntrl.thruster_error(False)
       return

   for cur_thrust in cntrl_msg.thruster_drives:
       if cur_thrust <0 or cur_thrust > g_tcntrl.esc_pwm_range:
          rospy.logerr("RF_ERROR: thrust provided (%s) out of configured range.", cur_thrust)
          tcntrl.thruster_error(False)
          return

   # Signal reception 
   debug_str = "Subscribed message recieved."
   rospy.loginfo(debug_str)

   for i in range(len(cntrl_msg.thrusters_to_update)):
      debug_str = "\t\tGPIO (%s) to new drive (%s)"
      rospy.loginfo(debug_str, cntrl_msg.thrusters_to_update[i], cntrl_msg.thruster_drives[i])

   if( g_tcntrl.update_ESCs(cntrl_msg.thrusters_to_update,
                            cntrl_msg.thruster_drives) == False ):
      rospy.logerr("RF_ERROR: update_ESCs failed")
      tcntrl.thruster_error(False)

def shutdown_hook():
   global g_tcntrl
   if g_tcntrl != None:
      g_tcntrl.pi.stop();


def listen_for_messages():
   """Setup thruster control subscriber"""
   global g_tcntrl
   g_tcntrl = ThrusterControl(ESC_GPIO_PINS,
                              ESC_PWM_FREQ,
                              ESC_PWM_RANGE)

   rospy.init_node("rf_thruster_control")
   
   #(topic),(custom message name),(name of callback function)
   rospy.Subscriber( "rf_thruster_control_topic",
                     RFThrusterControlData,
                     thruster_control_callback )

   # Register the ROS shutdown hook
   rospy.on_shutdown(shutdown_hook)

if __name__ == "__main__":
   listen_for_messages()
   rospy.spin()

