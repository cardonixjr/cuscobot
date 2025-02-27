#!/usr/bin/env python

from __future__ import print_function
import sys, select, termios, tty
import rospy
from std_msgs.msg import String

msg = """
Comandos Poter via teclado
---------------------------
Movimentos:
        w    
   a    s    d

Parada:
        p

Alterar velocidade:
c : aumenta
v : reduz

CTRL-C para sair
"""

class keyboardController():
    '''
    Class to read commands from keyboard and convert them into pwm commands for MG49 motors
    Uses tty and termios to read keyboard pressed keys and publish them into a ROS topic called "keyboard_input"
    '''

    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        print(msg)  

        ############################## ROS DEFINITION ##############################
        # Node name
        self.nodeName = "KeyboardReader"

        # Topic name
        self.topicName = "keyboard_input"

        # Init node
        rospy.init_node(self.nodeName, anonymous=True)
        self.nodeName = rospy.get_name()
        rospy.loginfo(f"The node - {self.nodeName} has started")

        # Init publisher
        self.keyPublisher = rospy.Publisher("keyboard_input", String, queue_size=5)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.keyPublisher.publish(key)
        return key

    def close(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)








class KeyToPWM():
    def __init__(self, base_pwm=128, pwm_dif=10, max_pwm=40):
        '''
        base_pwm: PWM that represents the speed zero;
        pwm_dif: PWM difference to slightly moves the robot
        max_pwm: Maximum difference from base_pwm
        '''

        self.max_pwm = max_pwm
        self.base_pwm = base_pwm
        self.pwm_dif = pwm_dif
        self.key = ""
        self.lastKey = ""

        self.setMoveBindings()
        self.speedBindings = {
            '=':(2),            # Increases speed
            '-':(-2)            # Decreases speed
        }
        self.key = ""

    def setMoveBindings(self):
        self.moveBindings = {
            'p':(self.base_pwm, 
                 self.base_pwm),                     # Stop

            'w':(self.base_pwm + self.pwm_dif, 
                 self.base_pwm + self.pwm_dif),      # Move forward

            's':(self.base_pwm - self.pwm_dif, 
                 self.base_pwm - self.pwm_dif),      # Move backward

            'd':(self.base_pwm + self.pwm_dif, 
                 self.base_pwm - self.pwm_dif),      # Turn right

            'a':(self.base_pwm - self.pwm_dif, 
                 self.base_pwm + self.pwm_dif)       # Turn left
        }

    def getSpeedByKey(self,key):
        if not key:
            print("undefined key")

        if key in self.speedBindings:
            if not (self.pwm_dif + self.speedBindings[key] > self.max_pwm) and not (self.pwm_dif + self.speedBindings[key] < 0):
                self.pwm_dif += self.speedBindings[key]
            
            key = self.lastKey
            self.setMoveBindings()
            print(f"speed set to {self.pwm_dif}")

        if key in self.moveBindings:
            self.lastKey = key
            return self.moveBindings[key][0], self.moveBindings[key][1]

        return self.base_pwm, self.base_pwm

if __name__ == "__main__":
    keyControl = keyboardController()
    #KeyConvert = KeyToPWM()
    
    while True:
        key = keyControl.getKey()
        print(f"key pressed: {key}")