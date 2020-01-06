#!/usr/bin/env python

import rospy
import paramiko
import sys
import re
import signal

import warnings # Suppressing annoying warning in paramiko
warnings.filterwarnings("ignore", category=FutureWarning)

from nova_common.msg import RadioStatus # Import custom msg

# Class representing the radio monitor
class RadioMonitor():

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # __init__():    #
  #    Initiases main class.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
  def __init__(self):

    rospy.init_node('radio_monitor')

    self.tmout = 0.5      # Timeout (in seconds) for ssh channel
    self.min_signal = -96 # Minimum signal strength, dB

    # Get package name for topic to publish radio status
    self.pkg_name = rospy.get_param("~pkg_name") 
    topic = '/' + self.pkg_name + '/radio_status'
    self.pub = rospy.Publisher(topic , RadioStatus, queue_size=1)

    self.initParams()
    self.radio_debug_msgs = [False, False]

    self.ssh = [] # Connect to both radios via ssh
    self.initRadioSSH()

    self.setupSig()

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # initParams():    #
  #    Initiase radio ROS parameters.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
  def initParams(self):

    if not rospy.has_param('/RadioDebug'):
      rospy.set_param('/RadioDebug', False)
      rospy.set_param('/RadioDebug9', False)
      rospy.set_param('/RadioDebug5', False)

    # Get radio authentication info from launch file
    self.hosts = [rospy.get_param("~radio_0/host"), rospy.get_param("~radio_1/host")]
    self.users = [rospy.get_param("~radio_0/user"), rospy.get_param("~radio_1/user")]
    self.pws   = [rospy.get_param("~radio_0/pw"),   rospy.get_param("~radio_1/pw")  ]

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # initRadioSSH():
  #    Initial connection to radios via ssh.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def initRadioSSH(self):
    
    for i in range(2):
      self.ssh.append(self.sshConnect(self.hosts[i], self.users[i], 
        self.pws[i], i))
    
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # getRadioMsgs():
  #    Getter for radio debug messages.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def getRadioMsgs(self):

    self.radio_debug_msgs[0] = rospy.get_param('/RadioDebug9')
    self.radio_debug_msgs[1] = rospy.get_param('/RadioDebug5')

    return self.radio_debug_msgs

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # setupSig():
  #    Set up sigint handler to close ssh sessions when Ctrl+C pressed
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def setupSig(self):

    def signal_handler(sig, frame):
      self.ssh[0].close() 
      self.ssh[1].close()
      rospy.signal_shutdown("SIGINT")
      
    signal.signal(signal.SIGINT, signal_handler) # Register sigint handler

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # sshConnect():
  #    Connect an SSH client to the SSH server running on the radio.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def sshConnect(self, host, user, pw, i):
    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
      ssh.connect(host, username=user, password=pw, timeout=self.tmout) 
      rospy.loginfo("Connected SSH client to radio " + str(i) + " at " + host + ".")   
    except:
      pass
      
    return ssh

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # isConnected():
  #    Check if the current SSH connection is active.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
  def isConnected(self, ssh):  
    try:
      ssh.exec_command('ls', timeout=self.tmout)
      return True    
    except:
      return False
  
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # getSignal():
  #    Read the signal strength from the radio.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
  def getSignal(self, ssh):   
    try:
      # Call mca-status to get radio information
      stdin, stdout, stderr = ssh.exec_command(
        "mca-status | grep signal", timeout=self.tmout)
      
      # Process resulting string to get value of signal strength      
      signal_strength = [int(s) for s in re.findall(r'[-]?[\d]+', stdout.read())][0]    
      return signal_strength
      
    except:
      return None

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # getNumWlanCons():
  #    Read the number of devices paired to the radio.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
  def getNumWlanCons(self, ssh):   
    try:
      # Call mca-status to get radio information
      stdin, stdout, stderr = ssh.exec_command(
        "mca-status | grep wlanConnections", timeout=self.tmout)
      
      # Process resulting string to get value of signal strength      
      n_wlan_cons = [int(s) for s in re.findall(r'[-]?[\d]+', stdout.read())][0]    
      return n_wlan_cons
      
    except:
      return None

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # checkRadios():
  #   Check the SSH and active connections of the radios.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
  def checkRadios(self):

    radio_msgs = [] # Two elements, representing whether each radio is up

    for i in range(2):
      # If ssh disconnected, reconnect and give error msg
      if not self.isConnected(self.ssh[i]): 
        #rospy.loginfo("SSH client cannot connect to radio " + str(i) + ".")
        self.ssh[i] = self.sshConnect(
          self.hosts[i], self.users[i], self.pws[i], i)
        
        msg = RadioStatus()
        msg.radio_id = i
        msg.signal = self.min_signal

        self.pub.publish(msg)    
        radio_msgs.append(False)
        
      # If ssh all gucci, grab and publish radio status information
      else:
        signal_strength = self.getSignal(self.ssh[i])
        n_wlan_cons = self.getNumWlanCons(self.ssh[i])
        
        if signal_strength is not None and n_wlan_cons is not None:
          msg = RadioStatus()
          msg.radio_id = i
          msg.n_wlan_cons = n_wlan_cons
          msg.signal = signal_strength
          msg.ssh_active = True

          self.pub.publish(msg)
          radio_msgs.append((n_wlan_cons>0) and msg.ssh_active)

        else:
          rospy.loginfo("SSH timeout on radio " + str(i) + ".")  

    return radio_msgs

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#   Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():   
  
  radio_monitor = RadioMonitor()  
  loop_hz = 1
  rate = rospy.Rate(loop_hz)
  
  while not rospy.is_shutdown():

    debug = rospy.get_param('/RadioDebug')

    if debug:
      radio_msgs = radio_monitor.getRadioMsgs()
    else:
      radio_msgs = radio_monitor.checkRadios()

    #print radio_msgs[0]
    #print radio_msgs[1]
    


    rate.sleep()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
