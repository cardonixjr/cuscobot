import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster
from math import cos, sin, pi

class DeadReckoningOdom():
	def _init__(self):
		# Node name to this class
		self.nodeName = "DeadReckoningOdom"

		# Topic names
		self.topicNameLeftEncoder = "left_encoder_pulses"
		self.topicNameRightEncoder = "right_encoder_pulses"

		# Frame names
		self.baseFrameName = rospy.get_param('~base_frame_id', 'base_frame')
		self.odomFrameName = rospy.get_param('~odom_frame_id', 'odom_frame')

		# Robot geometry
		self.wheelRadius = 0.0326
		self.distanceWheels = 0.13

		# Encoder
		self.encoderPulsesConstant = 20

		# update frequency fom publishing odom in Hz
		self.updateFrequencyPublish = 10
		
		# flag for initial reading - these variables are set to zero after
		# the first reading from the encoder topics
		self.flagInitialLeftEncoder=1
		self.flagInitialRightEncoder=1
		
		# Variables to store "trash" readings when the encoders initiate
		self.initialvalueLeftEncoder = 0
		self.initialValueRightEncoder = 0

		# Encoder readings
		self.currentValueLeftEncoder = 0
		self.pastValueLeftEncoder = 0

		self.currentValueRightEncoder = 0
		self.pastValueRightEncoder = 0

		# Time readings
		self.currentTimeLeftEncoder = 0
		self.currentTimeRightEncoder = 0

		self.currentTime = 0
		self.pastTime = 0

		self.x = 0
		self.y = 0
		self.theta = 0


		#### init node ####
		rospy.init_node(self.nodeName, anonymous = True)

		# Get time
		self.psatTime = rospy.get_time()
		self.nodeName = rospy.get_name()

		rospy.loginfo(f"The node - {self.nodeName} has started")


		#### Subscribers and publishers ####
		rospy.Subscriber(self.topicNameLeftEncoder, Int32, self.callBackFunctionLeftEncoder)
		rospy.Subscriber(self.topicNameRightEncoder, Int32, self.callBackFunctionRightEncoder)

		# publish odometry
		self.odometryPyblisher = rospy.Publisher("Odom", Odometry, queue_size=5)
		self.odometryBroadcaster = TransformBroadcaster()

	def callBackFunctionLeftEncoder(self, message1):
		if self.flagInitialLeftEncoder == 1:
			self.initialValueLeftEncoder = message1.data
			self.flagInitialLeftEncoder = 0
		else:
			self.currentValueLeftEncoder = message1.data - self.initialvalueLeftEncoder
			self.currentTimeLeftEncoder = rospy.get_time()

	def callBackFunctionRightEncoder(self, message2):
		if self.flagInitialRightEncoder == 1:
			self.initialValueRightEncoder = message2.data
			self.flagInitialRightEncoder = 0
		else:
			self.currentValueRightEncoder = message2.data - self.initialvalueRightEncoder
			self.currentTimeRightEncoder = rospy.get_time()
			
	def calculateUpdate(self):
		self.currentTime = rospy.get_time()

		if currentTime > self.pastTime:
			deltaT = self.currentTime - self.pastTime

			# calcula a variação dos angulos

			# calcula a velocidade angular dos encoders

			# calcula as velocidades dos motores

			# calcula a velocidade angular do robô

			# calcula o dead reckoning
				# distance traveled
				# angle change

				# x
				# y
				# theta
		
		# prints

		# Create a quaternion object and compute the values
		quaternion = Quaternion()

		# Create a data structure for the quaternion		
		quaternion1 = Quaternion()
		quaternion1.x = 0
		quaternion1.y = 0
		quaternoin1.z = sin(self.theta/2)
		quaternion1.w = cos(self.theta/2)
		quaternionTuple = (quaternion1.x, 
						   quaternion1.y, 
						   quaternion1.z,
						   quaternion1.w)

		# publish odometry
		self.odometryBroadcaster.sendTransform((self.x, self.y, 0),
												quaternionTuple,
												rospy.Time.now(),
												self.baseFrameName,
												self.odomFrameName)

		odometry1 = Odometry()
		odometry1.header.stamp = rospy.Time.now()
		odometry1.header.frame_id = self.odomFrameName
		odometry1.pose.pose.position.x = self.x
		odometry1.pose.pose.position.y = self.y
		odometry1.pose.pose.position.z = 0
		odometry1.pose.pose.orientation = quaternion1
		odometry1.child_frame_id = self.baseFrameName
		odometry1.twist.twist.linear.x = velocity
		odometry1.twist.twist.linear.y = 0
		odometry1.twist.twist.angular.z = angularVelocity

		self.odometryPublisher.publish(odometry1)

		self.pastTime.self.currentTime
		self.pastValueLeftEncoder = self.currentvalueLeftEncoder
		self.pastValueRightEncoder = self.currentValueRightEncoder


	def mainLoop(self):
        ROSRate= rospy.Rate(self.updateFrequencyPublish)
        
        while not rospy.is_shutdown():
            self.calculateUpdate()
            ROSRate.sleep()
   

if __name__ == "__main__":
   	""" main """
    objectDR = DeadReckoningOdom()
    objectDR.mainLoop()