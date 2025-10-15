# ROS
import rospy
from PRESETS import *
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarDetection():
    def __init__(self):
        # Update Frequency
        self.updateFrequencyPublish = UPDATE_FREQUENCY

        # Messages
        self.left_vib_pwm = 0
        self.right_vib_pwm = 0
        self.scan = None

        # ROS Node name to this class
        self.nodeName = "LidarVib"

        # ROS Topic names
        self.topicNameLeftVib = "left_vib_msg"
        self.topicNameRightVib = "right_vib_msg"
        self.topicNameLidar = "scan"

        # ROS node
        rospy.init_node(self.nodeName, anonymous = True)
        self.nodeName = rospy.get_name()
        rospy.loginfo(f"The node - {self.nodeName} has started")

        # Subscribers
        rospy.Subscriber(self.topicNameLidar, LaserScan, self.callbackFunctionLidar)
        
        # Publishers
        self.left_vib_pub = rospy.Publisher(self.topicNameLeftVib, Int32, queue_size=5)
        self.right_vib_pub = rospy.Publisher(self.topicNameRightVib, Int32, queue_size=5)

        # Rate publisher
        self.ratePublisher = rospy.Rate(self.updateFrequencyPublish)

    def callbackFunctionLidar(self, message):
        if message:
            #print(type(message))
            self.scan = message

    def calculateVib(self):
        if self.scan:
            # recebe valor do SCAN e calcula os PWMs 
 
#            for laser in self.scan.ranges[240:479]:
#                if laser < 100:
#                    self.left_vib_pwm = 15
#                    self.right_vib_pwm = 15
#                if laser < 60:
#                    self.left_vib_pwm = 64
#                    self.right_vib_pwm = 64
#                if laser < 30:
#                    self.left_vib_pwm = 128
#                    self.right_vib_pwm = 128
#                #else:
#                #    self.left_vib_pwm=0
#                #    self.right_vib_pwm=0
#
#            for laser in self.scan.ranges[:239]:
#                if laser < 100:
#                    self.left_vib_pwm = 30
#                if laser < 60:
#                    self.left_vib_pwm = 128
#                if laser < 30:
#                    self.left_vib_pwm = 256
#
#                else:
#                    self.left_vib_pwm=0
#
#            for laser in self.scan.ranges[480:719]:
#                if laser < 100:
#                    self.right_vib_pwm = 30
#                if laser < 60:
#                    self.right_vib_pwm = 128
#                if laser < 30:
#                    self.right_vib_pwm = 256
#                else:
#                    self.right_vib_pwm=0

            #print(573 +- 143)

            l_nearest = 100000
            for laser in self.scan.ranges[574:717]:
                if laser < l_nearest:
                    l_nearest = laser

            if l_nearest < 1:
                self.left_vib_pwm = 30
                if l_nearest < 0.6:
                    self.left_vib_pwm = 64
                    if l_nearest < 0.4:
                        self.left_vib_pwm = 128
            else:
                self.left_vib_pwm=0

            r_nearest = 10000
            for laser in self.scan.ranges[430:573]:
                if laser < r_nearest:
                    r_nearest = laser

            if r_nearest < 1:
                self.right_vib_pwm = 30
                if r_nearest < 0.6:
                    self.right_vib_pwm = 64
                    if r_nearest < 0.4:
                        self.right_vib_pwm = 128
            else:
                self.right_vib_pwm=0

            #print(self.scan.ranges)

        # Publish
        self.left_vib_pub.publish(self.left_vib_pwm)
        self.right_vib_pub.publish(self.right_vib_pwm)

    def mainLoop(self):
        try:
            while not rospy.is_shutdown():
                self.calculateVib()
                self.ratePublisher.sleep()

        except Exception as e:
            print(e)

        finally:
            # End of execution
            pass

if __name__ == "__main__":
    """ main """
    objectVB = LidarDetection()
    objectVB.mainLoop()

