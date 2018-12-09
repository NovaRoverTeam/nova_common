#!/usr/bin/env python

import rospy
import paramiko
import sys
import re
import signal

import warnings # Suppressing annoying warning in paramiko
warnings.filterwarnings("ignore", category=FutureWarning)

from radio_sm import RadioStateMachine
from nova_common.msg import RadioStatus # Import custom msg

loop_hz = 1      # ROS loop rate in Hz, coupled with tmout time below
tmout = 0.5      # Timeout (in seconds) for ssh channel
min_signal = -96 # Minimum signal strength, dB

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# sshConnect():
#    Connect an SSH client to the SSH server running on the radio.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def sshConnect(host, user, pw, i):
  ssh = paramiko.SSHClient()
  ssh.load_system_host_keys()
  ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
  
  try:
    ssh.connect(host, username=user, password=pw, timeout=tmout) 
    rospy.loginfo("Connected SSH client to radio " + str(i) + " at " + host + ".")   
  except:
    pass
    
  return ssh

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# isConnected():
#    Check if the current SSH connection is active.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def isConnected(ssh):  
  try:
    ssh.exec_command('ls', timeout=tmout)
    return True    
  except:
    return False
     
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# getSignal():
#    Read the signal strength from the radio.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def getSignal(ssh):   
  try:
    # Call mca-status to get radio information
    stdin, stdout, stderr = ssh.exec_command("mca-status | grep signal", timeout=tmout)
    
    # Process resulting string to get value of signal strength      
    signal_strength = [int(s) for s in re.findall(r'[-]?[\d]+', stdout.read())][0]    
    return signal_strength
    
  except:
    return None

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# getNumWlanCons():
#    Read the number of devices paired to the radio.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def getNumWlanCons(ssh):   
  try:
    # Call mca-status to get radio information
    stdin, stdout, stderr = ssh.exec_command("mca-status | grep wlanConnections", timeout=tmout)
    
    # Process resulting string to get value of signal strength      
    n_wlan_cons = [int(s) for s in re.findall(r'[-]?[\d]+', stdout.read())][0]    
    return n_wlan_cons
    
  except:
    return None
  
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# updateStateMachine():
#   Update the state machine with the current radio status.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def updateStateMachine(radio_sm, radio_msgs):

  #rospy.loginfo("5 is " + str(radio_msgs[0]) + " and 9 is " + str(radio_msgs[1]))

  if   radio_msgs[0] is False and radio_msgs[1] is False:
    radio_sm.downBoth()
  elif radio_msgs[0] is False and radio_msgs[1] is True:
    radio_sm.up9()
  elif radio_msgs[0] is True and radio_msgs[1] is False:
    radio_sm.up5()
  elif radio_msgs[0] is True and radio_msgs[1] is True:
    radio_sm.upBoth()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#   Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():   
  rospy.init_node('radio_monitor')
  rate = rospy.Rate(loop_hz)
  
  pkg_name = rospy.get_param("~pkg_name") # Get package name for topic
  topic = '/' + pkg_name + '/radio_status'
  pub = rospy.Publisher(topic , RadioStatus, queue_size=1)
  
  # Get radio authentication info from launch file
  hosts = [rospy.get_param("~radio_0/host"), rospy.get_param("~radio_1/host")]
  users = [rospy.get_param("~radio_0/user"), rospy.get_param("~radio_1/user")]
  pws   = [rospy.get_param("~radio_0/pw"),   rospy.get_param("~radio_1/pw")  ]
    
  ssh = [] # Connect to both radios via ssh
  for i in range(2):
    ssh.append(sshConnect(hosts[i], users[i], pws[i], i))
  
  # Set up sigint handler to close ssh sessions when Ctrl+C pressed
  def signal_handler(sig, frame):
    ssh[0].close() 
    ssh[1].close()
    rospy.signal_shutdown("SIGINT")
    
  signal.signal(signal.SIGINT, signal_handler) # Register sigint handler

  radio_sm = RadioStateMachine(pkg_name)
  
  while not rospy.is_shutdown():

    radio_msgs = [] # Store the status of the 2 radios
  
    for i in range(2):
      # If ssh disconnected, reconnect and give error msg
      if not isConnected(ssh[i]): 
        #rospy.loginfo("SSH client cannot connect to radio " + str(i) + ".")
        ssh[i] = sshConnect(hosts[i], users[i], pws[i], i)
        
        msg = RadioStatus()
        msg.radio_id = i
        msg.signal = min_signal

        pub.publish(msg)    
        radio_msgs.append(False)
	      
      # If ssh all gucci, grab and publish radio status information
      else:
        signal_strength = getSignal(ssh[i])
        n_wlan_cons = getNumWlanCons(ssh[i])
        
        if signal_strength is not None and n_wlan_cons is not None:
          msg = RadioStatus()
          msg.radio_id = i
          msg.n_wlan_cons = n_wlan_cons
          msg.signal = signal_strength
          msg.ssh_active = True

          pub.publish(msg)
          radio_msgs.append((n_wlan_cons>0) and msg.ssh_active)

        else:
          rospy.loginfo("SSH timeout on radio " + str(i) + ".")  

    # Update state machine with radio status
    updateStateMachine(radio_sm, radio_msgs)

    rate.sleep()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
