#!/usr/bin/env python3

# ROS
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String, Int32, Empty
from nav_msgs.msg import Odometry as Odom
import tf

# Other project classes and functions
import encoder, odom_utls
from PRESETS import *

# Plotting
import matplotlib.pyplot as plt
import plotly.graph_objects as go

# Aux
import numpy as np
import time

# Plotting
PLOTTING = True

class DeadReckoningOdom():
    '''
        Main class of the project. Implements all ROS pubs/subs and handle the odometry, keyboard control and arduino communication
    '''

    def __init__(self):
        # Update Frequency
        self.updateFrequencyPublish = UPDATE_FREQUENCY

        # Robot geometry
        self.wheelRadius = WHEEL_RADIUS
        self.wheelBase = WHEEL_BASE

        # Encoders
        self.left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
        self.right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)

        # Odometry
        self.odom = odom_utls.odometry(self.left_wheel_encoder, self.right_wheel_encoder, self.wheelBase, self.wheelRadius)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_theta = 0
        self.v = 0           # linear speed in X axis
        self.w = 0           # angular velocity

        # Time stamps
        self.start_time = time.time_ns()
        self.last_read = time.time()

        # aux
        self.is_first_loop = True
        self.executeOnce = True
        self.left_pwm = 128
        self.right_pwm = 128

        # Plotting
        if PLOTTING:
            self.pose_log = {'x':[], 'y':[], 'theta':[]}
            self.fig, self.ax = plt.subplots()
            self.line = self.ax.scatter(self.pose_log['x'], self.pose_log['y'])

            plt.axis([-2,2,-2,2])
            plt.show(block=False)
            plt.pause(.1)

        ############################## ROS DEFINITION ##############################
        # ROS Node name to this class
        self.nodeName = "DeadReckoningOdom"
        # ROS Topic names
        self.topicNameLeftEncoder = "left_encoder_pulses"
        self.topicNameRightEncoder = "right_encoder_pulses"
        self.topicNameResetEncoder = "reset_encoder"

        # ROS node
        rospy.init_node(self.nodeName, anonymous = True)
        self.nodeName = rospy.get_name()
        rospy.loginfo(f"The node - {self.nodeName} has started")

        # Subscribers
        rospy.Subscriber(self.topicNameLeftEncoder, Int32, self.callBackFunctionLeftEncoder)
        rospy.Subscriber(self.topicNameRightEncoder, Int32, self.callBackFunctionRightEncoder)

        # Publishers
        self.resetPublisher = rospy.Publisher(self.topicNameResetEncoder, Empty, queue_size = 1)

        # Odometry publishers
        self.odom_pub = rospy.Publisher("odom", Odom, queue_size=20)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Rate publisher
        self.ratePublisher = rospy.Rate(self.updateFrequencyPublish)

        # Reset encoders
        time.sleep(2)
        self.resetPublisher.publish()

    def callBackFunctionLeftEncoder(self, message1):
        ''' Callback function called when "left_encoder_pulses" topic receive a message'''
        if message1.data:
            self.left_wheel_encoder.counter = message1.data

    def callBackFunctionRightEncoder(self, message2):
        ''' Callback function called when "right_encoder_pulses" topic receive a message'''
        if message2.data:
            self.right_wheel_encoder.counter = message2.data

    def calculateUpdate(self):
        ''' 
            Looped function
            Call the odometry function to gets the actual robot coordinates
            Gets the keyboard commands and publish motors PWM
            Build the ROS Odom message and publish it to Odom topic
        '''

        ############################## ODOMETRY ##############################
        # Calculate how many ns passed since last read
        t = time.time_ns()
        dt = t - self.start_time
        self.start_time = t
        current_time = rospy.Time.now()

        # Run odometry step to update robot location
        self.odom.step(dt)
        self.x, self.y, self.theta = self.odom.getPose()
        print(f"x: {self.x}, y:{self.y}, t:{self.theta}")

        # Get speed values
        self.v, self.w = self.odom.getSpeed()
        print(f"v: {self.v}, w: {self.w}")

        # Built Odom and transform messages
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.theta)

        odom = Odom()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.v, 0, 0), Vector3(0,0,self.w))

        # Reset variables
        self.last_theta = self.theta

        ############################## ROS PUBLISHERS ##############################
        # publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # publish Odom
        self.odom_pub.publish(odom)

        ############################## PLOTTING ##############################
        if PLOTTING:
            self.pose_log['x'].append(self.x)
            self.pose_log['y'].append(self.y)
            self.pose_log['theta'].append(self.theta)

            # Add a plot
            self.line.set_offsets(np.c_[self.pose_log['x'], self.pose_log['y']])
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def mainLoop(self):

        try:
            while not rospy.is_shutdown():
                self.calculateUpdate()
                self.ratePublisher.sleep()

        except Exception as e:
            print(e)

        finally:
            # End of execution
            self.resetPublisher.publish()
            #self.stop()

            fig = go.Figure()
            fig.add_trace(go.Scatter(x=self.pose_log['x'], y=self.pose_log['y'], mode='lines+markers', name='Trajetória'))
            fig.update_layout(title='Trajetória do Robô Móvel', xaxis_title='Posição X (metros)', yaxis_title='Posição Y (metros)')
            # Exibe o gráfico no navegador
            fig.show()


if __name__ == "__main__":
    """ main """
    objectDR = DeadReckoningOdom()
    objectDR.mainLoop()

