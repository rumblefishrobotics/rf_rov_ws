#!/usr/bin/env python3
import pigpio
import rospy
import time

from rf_common.msg import RFThrusterControlData

# Resources
# http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks

#-------------------------------------------------------------------------------
# Control Constants
#-------------------------------------------------------------------------------
ESC_GPIO_PINS = [4,17,18,27]   

# Signal frequency in HZ ((2000us * 50) = 1S)
ESC_PWM_FREQ = 50

# Number of steps in PWM control
ESC_PWM_RANGE = 40000

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
      self.esc_pwm_freq  = esc_pwm_freq
      self.esc_pwm_range = esc_pwm_range  
      self.gpio_pin_nos  = gpio_pin_nos

      self.pi = pigpio.pi()
      
      # ESCs configured for a full stick range of 1040-1960mS control pulse on a
      # 50Hz (20mS) signal. 

      self.pwm_max = int((1960 / 20000 ) * self.esc_pwm_range)
      self.pwm_min = int((1040 / 20000 ) * self.esc_pwm_range)
      self.pwm_neutral = int((self.pwm_max + self.pwm_min) / 2)
      
      # pdrives stores the drive % value for each ESC pin. To start
      # the ESCs need to see a "locked" signal with the control line
      # signalling 50% or idle. 
      self.pdrives = [self.pwm_neutral] * len(ESC_GPIO_PINS)

      for cur_pin_no in self.gpio_pin_nos:
         self.pi.set_PWM_frequency( cur_pin_no, self.esc_pwm_freq )
         self.pi.set_PWM_range( cur_pin_no, self.esc_pwm_range )
         self.pi.set_PWM_dutycycle( cur_pin_no, self.pwm_neutral )

      # Disarm ESCs. 
      self.disarm_ESCs()
      return

   #--------------------------------------------------------
   #
   #--------------------------------------------------------   
   def disarm_ESCs(self):
      for i in range(len(ESC_GPIO_PINS)):
         cur_pin_no = ESC_GPIO_PINS[i]
         self.pi.set_PWM_dutycycle(cur_pin_no, self.pwm_max)
      time.sleep(0.5)

      for i in range(len(ESC_GPIO_PINS)):
         cur_pin_no = ESC_GPIO_PINS[i]
         self.pi.set_PWM_dutycycle(cur_pin_no, self.pwm_min)
      time.sleep(0.5)

      # After they are disarmed leave the ESCs in neutral
      for i in range(len(ESC_GPIO_PINS)):
         cur_pin_no = ESC_GPIO_PINS[i]
         self.pi.set_PWM_dutycycle(cur_pin_no, self.pwm_neutral)

      
   #-------------------------------------------------------------------------------
   # ESC thrust output giving in range -100 - +100 to represent precent drive and
   # direction. 
   #-------------------------------------------------------------------------------
   def update_ESCs(self, ESCs_to_update, drive_updates):
      if len(ESCs_to_update) != len(drive_updates):
         return False
      
      # Perform basic sanity checks
      for i in range(len(ESCs_to_update)):
         cur_pin_no = ESCs_to_update[i]
         cur_pdrive = drive_updates[i]
         if( cur_pin_no not in ESC_GPIO_PINS ):
            return False
         if( cur_pdrive < -100 or cur_pdrive > 100):
            return False
         
      for i in range(len(ESCs_to_update)):
         cur_pin_no = ESCs_to_update[i]
         cur_pdrive = drive_updates[i]

         # Convert drive from % to PWM duty cycle.
         if(cur_pdrive < 0):
            cur_pdrive = (cur_pdrive * (self.pwm_neutral - self.pwm_min))/100
         elif(cur_pdrive > 0):
            cur_pdrive = (cur_pdrive * (self.pwm_max - self.pwm_neutral))/100
         cur_pdrive += self.pwm_neutral
         
         log_str = "\t\tupdate_ESCs testing GPIO (%s) to new drive (%s)"
         rospy.loginfo(log_str, cur_pin_no, cur_pdrive)
         self.pi.set_PWM_dutycycle(cur_pin_no, cur_pdrive)

      return True
      
   #--------------------------------------------------------
   #
   #--------------------------------------------------------
   def ESCs_to_neutral(self):
      """
          Places all ESCs to neutral / locked position to stop the motors. 
      """
      for cur_pin_no in ESC_GPIO_PINS:
         range = self.pi.get_PWM_range( cur_pin_no )
         self.pi.set_PWM_dutycycle( cur_pin_no, (range * 0.5) )

   #--------------------------------------------------------
   #
   #--------------------------------------------------------
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

   #--------------------------------------------------------
   # Regisered with ROS as the callback on incommining 
   # messages of theRFThrusterControlData message types. 
   #--------------------------------------------------------

   def thruster_control_callback(self, cntrl_msg):
      """
       Callback message is a list of thrusters specified as Broadcom GPIO pin 
       numbers, and thrust at which to drive them. Thrusts specified as a positive
       integer in the drive range. 
      """
   
      if(len(cntrl_msg.thrusters_to_update) != len(cntrl_msg.thruster_drives)):
         rospy.logerr("RF_ERROR: Bad thruser control message, (thrusters != thrusts)")
         tcntrl.thruster_error(False)
         return
      
      for cur_thrust in cntrl_msg.thruster_drives:
         if cur_thrust <-100 or cur_thrust > 100:
            rospy.logerr("RF_ERROR: thrust provided (%s) out of configured range.", cur_thrust)
            tcntrl.thruster_error(False)
            return
         
         # Signal reception 
         rospy.loginfo("Subscribed message recieved.")
      
         for i in range(len(cntrl_msg.thrusters_to_update)):
            rospy.loginfo("\t\tGPIO (%s) to new drive (%s)",
                          cntrl_msg.thrusters_to_update[i],
                          cntrl_msg.thruster_drives[i])
            
            if( self.update_ESCs(cntrl_msg.thrusters_to_update,
                                 cntrl_msg.thruster_drives) == False ):
               rospy.logerr("RF_ERROR: update_ESCs failed")
               tcntrl.thruster_error(False)
      
   #--------------------------------------------------------
   # Shutdown Hook
   #--------------------------------------------------------
   def shutdown_hook(self):
      self.pi.stop();

#-------------------------------------------------------------------------------
# End of ThrusterControl Class Definition
#-------------------------------------------------------------------------------

   
#-------------------------------------------------------------------------------
# ROS setup below here
#-------------------------------------------------------------------------------
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
                     g_tcntrl.thruster_control_callback )

   # Register the ROS shutdown hook
   rospy.on_shutdown(g_tcntrl.shutdown_hook)
   
#-------------------------------------------------------------------------------
# Python Launch Singleton
#-------------------------------------------------------------------------------
if __name__ == "__main__":
   listen_for_messages()
   rospy.spin()

