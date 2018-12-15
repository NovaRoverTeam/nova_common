# nova_common
Provides common messages, services and utilities for the rover and base station.

## Dependencies

- Python module *transitions*:
```
pip install transitions
```

## Multi-Machine Launch Files

*NOTE: Currently looking at replacing this multi-machine system with the multimaster_fkie package.*

The file system.launch requires connection to the rover computer in order to launch nodes remotely. To do this, an ssh connection must be established to the other computer by roslaunch. In order for this to be automated by roslaunch, you must generate an ssh key pair on the rover, and add its public key to the ~/.ssh/authorized_keys file on the base station computer. Also, make sure the rover computer has the base station computer's hostname paired to its static IP address in the rover's /etc/hosts file, and vice versa for the base station computer's /etc/hosts file.

If the following issue arises:
  
  ```"Unable to establish ssh connection to [user@hostname:22]: Server u'hostname' not found in known_hosts"```

Please do the following:
  
  - Remove the rover's hostkey from the base station's ~/.ssh/known_hosts file:
  
  ```ssh-keygen -R hostname```
  - In a new terminal, set up a single ssh connection to the rover with the following additional option:
  
  ```ssh user@hostname -oHostKeyAlgorithms='ssh-rsa'```
  - Exit the ssh session. The launch file should be able to set up the ssh connection by itself now.
  
The parts of the system.launch file that are key for the system to work are the \<env\> tag setting the ROS master URI, the defining of \<group\> tags with the \<machine\> tag set for the rover group, and the "env-loader" field in the \<machine\> tag being set to that of the catkin workspace being used on the rover.
 
