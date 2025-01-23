#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

nodeName = "messagepublisher"

topicName = "arduino_command"

rospy.init_node(nodeName, anonymous=True)

commandPublisher = rospy.Publisher(topicName, Int32, queue_size=5)

ratePublisher = rospy.Rate(1)

command = "a128"

while not rospy.is_shutdown():
	rospy.loginfo(command)
	
	
	
	commandPublisher.publish(command)
	ratePublisher.sleep()
