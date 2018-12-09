#!/usr/bin/env python

import rospy
from transitions import Machine

# Class representation of state machine
class RadioStateMachine():

    states = ['Connected', '5_Loss', '9_Loss', 'Full_Loss']

    def __init__(self, pkg_name):

        self.pkg_name = pkg_name # Name of pkg, core_rover or base_station

        # Transitions between the states. 'trigger' is a function you can call
        # on the class object which triggers the transition to occur. The
        # functions listed in 'before' are functions within the class that will 
        # run just after the transition takes place. Calling the function returns
        # the transition's success or lack thereof.
        transitions = [
            { 'trigger': 'upBoth', 'source': 'Full_Loss', 'dest': 'Connected', 
                'after':['connected', 'camRestore']},
            { 'trigger': 'up5',    'source': 'Full_Loss', 'dest': '9_Loss', 
                'after':['loss9', 'camRestore']},
            { 'trigger': 'up9',    'source': 'Full_Loss', 'dest': '5_Loss', 
                'after':['loss5', 'camBackup']},

            { 'trigger': 'downBoth', 'source': 'Connected', 'dest': 'Full_Loss', 
                'after':['fullLoss', 'camShutdown']},
            { 'trigger': 'up9',      'source': 'Connected', 'dest': '5_Loss', 
                'after':['loss5', 'camBackup']},
            { 'trigger': 'up5',      'source': 'Connected', 'dest': '9_Loss', 
                'after':['loss9', 'camRestore']},

            { 'trigger': 'downBoth', 'source': '5_Loss', 'dest': 'Full_Loss', 
                'after':['fullLoss', 'camShutdown']},
            { 'trigger': 'upBoth',   'source': '5_Loss', 'dest': 'Connected', 
                'after':['connected', 'camRestore']},
            { 'trigger': 'up5',      'source': '5_Loss', 'dest': '9_Loss', 
                'after':['loss9', 'camRestore']},

            { 'trigger': 'downBoth', 'source': '9_Loss', 'dest': 'Full_Loss', 
                'after':['fullLoss', 'camShutdown']},
            { 'trigger': 'upBoth',   'source': '9_Loss', 'dest': 'Connected', 
                'after':['loss9', 'camRestore']},
            { 'trigger': 'up9',      'source': '9_Loss', 'dest': '5_Loss', 
                'after':['loss5', 'camBackup']}
        ]

        # Initialize the state machine
        self.mach = Machine(model=self, 
            states=RadioStateMachine.states, initial='Connected',
            transitions=transitions, ignore_invalid_triggers=True)

        # Initialise the global state parameter
        self.setConnectivity('Connected')

    def setConnectivity(self, state):
        # TODO uncomment this - it's just for testing the system without radios
        #rospy.set_param('/' + self.pkg_name + '/Connectivity', state)
        rospy.set_param('/' + self.pkg_name + '/Connectivity', 'Connected')

    def updateGuiRadioState(self, message):
        # TODO make service call to GUI to display message re: state change
        rospy.loginfo(message)

    def connected(self):
        message = "Reconnected all radios."
        self.updateGuiRadioState(message)
        self.setConnectivity('Connected')

    def loss5(self):
        message = "Lost only the 5.8 GHz radio connection."
        self.updateGuiRadioState(message)
        self.setConnectivity('5_Loss')

    def loss9(self):
        message = "Lost only the 900 MHz radio connection."
        self.updateGuiRadioState(message)
        self.setConnectivity('9_Loss')

    def fullLoss(self):
        message = "Lost all radio connections."
        self.updateGuiRadioState(message)
        self.setConnectivity('Full_Loss')

    def camShutdown(self):
        # TODO service call to close all camera streams
        pass

    def camBackup(self):
        # TODO service call to disable all cams and stream designated backup over 900
        pass

    def camRestore(self):
        # TODO service call to restore all camera streams over 5.8
        pass