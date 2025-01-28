import rospy
from std_msgs.msg import Int32, String
import time
# import keyboard
import matplotlib.pyplot as plt
import numpy as np
import encoder, odometry, serialCom, goToGoal

import plotly.graph_objects as go
import matplotlib.pyplot as plt

# MG49 commands
SET_SPEED_RIGHT = chr(100)
SET_SPEED_LEFT = chr(101)
SET_SPEED = chr(102)
GET_LEFT_ENCODER = chr(105)
GET_RIGHT_ENCODER = chr(106)
RESET_ENCODER = chr(114)

# Robot info
WHEEL_RADIUS = 0.06
WHEEL_BASE = 0.37
TICKS_PER_REVOLUTION = 980
MAX_PWM = 48
MAX_PWM_STEP = 10 # Maior mudança de PWM a cada loop. Isso igual a MAX_PWM é basicamente sem limitação de tamanho de passo.
MAX_SPEED_DISTANCE = 1 # Distância em metros antes do robô começar a reduzir a velocidad

PLOTTING = True

PATH = [[1,1]]
step = 0
goal = PATH[step]

class DeadReckoningOdom():
    def __init__(self):
        # Node name to this class
        self.nodeName = "DeadReckoningOdom"

        # Topic names
        self.topicNameLeftEncoder = "left_encoder_pulses"
        self.topicNameRightEncoder = "right_encoder_pulses"
        self.topicNamePublisher = "arduino_command"

        # Path
        self.goal = goal
        self.path = PATH
        self.step = step

        # Speed limits
        self.max_speed_distance = MAX_SPEED_DISTANCE
        self.max_pwm = MAX_PWM

        # Robot geometry
        self.wheelRadius = WHEEL_RADIUS
        self.wheelBase = WHEEL_BASE

        # Encoder
        self.left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
        self.right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)

        # update frequency fom publishing odom in Hz
        self.updateFrequencyPublish = 1

        # Odometry
        self.odom = odometry.odometry(self.left_wheel_encoder, self.right_wheel_encoder, self.wheelBase)
        self.x = 0
        self.y = 0
        self.theta = 0

        # PID Controller
        self.controller = goToGoal.GoToGoal()
        self.w = 0
        self.last_w = 0

        # aux
        self.start_time = time.time_ns()
        self.last_read = time.time()

        self.command = ""

        self.last_left_pwm = 128
        self.last_right_pwm = 128

        # Plotting
        if PLOTTING:
            self.pose_log = {'x':[], 'y':[], 'theta':[]}
            self.fig, self.ax = plt.subplots()
            self.line = self.ax.scatter(self.pose_log['x'], self.pose_log['y'])

            plt.axis([-2,2,-2,2])
            plt.show(block=False)
            plt.pause(.1)


        ########## ROS DEFINITION ##########
        # ROS node
        rospy.init_node(self.nodeName, anonymous = True)
        self.nodeName = rospy.get_name()
        rospy.loginfo(f"The node - {self.nodeName} has started")

        # Subscribers for receiving encoder readings
        rospy.Subscriber("left_encoder_pulses", Int32, self.callBackFunctionLeftEncoder)
        rospy.Subscriber("right_encoder_pulses", Int32, self.callBackFunctionRightEncoder)
        
        self.leftPublisher = rospy.Publisher("left_wheel_pwm", Int32, queue_size=5)
        self.rightPublisher = rospy.Publisher("right_wheel_pwm", Int32, queue_size=5)

        # Rate publisher
        self.ratePublisher = rospy.Rate(self.updateFrequencyPublish)

    def callBackFunctionLeftEncoder(self, message1):
        if message1.data:
            self.left_wheel_encoder.counter = message1.data

    def callBackFunctionRightEncoder(self, message2):
        if message2.data:
            self.right_wheel_encoder.counter = message2.data

    def calculateUpdate(self):
        # Calculate how many ns passed since last read
        t = time.time_ns()
        dt = t - self.start_time
        self.start_time = t

        # Run odometry step to update robot location
        self.odom.step()
        self.x, self.y, self.theta = self.odom.getPose()

        # Log into plot map
        if PLOTTING:
            self.pose_log['x'].append(self.x)
            self.pose_log['y'].append(self.y)
            self.pose_log['theta'].append(self.theta)

            # Add a plot
            self.line.set_offsets(np.c_[self.pose_log['x'], self.pose_log['y']])
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            print(f"x: {self.x}, y:{self.y}, t:{self.theta}")

        # Calculates the angular speed w
        # self.w = self.controller.step(self.goal[0], self.goal[1], self.x, self.y, self.theta, dt, precision = 0.05)

        # if self.w != None: self.last_w = self.w
        # if self.w == None: self.w = self.last_w
        # print(f"velocidade angular calculada: {self.w}")

        # left, right = odometry.uni_to_diff(5, self.w, self.left_wheel_encoder, self.right_wheel_encoder, self.wheelBase)

        # Normalize the result speed
        # if left > right:
        #     left_norm = left/left
        #     right_norm = right/left
        # else:
        #     left_norm = left/right
        #     right_norm = right/right

        # max_speed = self.controller.speed_limit_by_distance(self.max_speed_distance, self.max_pwm, self.goal[0], self.goal[1], self.x, self.y)
        # left_pwm = left_norm*max_speed
        # right_pwm = right_norm*max_speed

        # The MG49 Driver reads the speed in range 0 - 255. Values greater than 128 are "positive" speeds,
        # while values between 0 and 128 are the negative ones. So, the 128 must be considered the speed 0.
        # left_pwm += 128
        # right_pwm += 128

        # Take a little step in the direction of the speed calculated by the Controller
        # This is made to prevent a huge speed change in a small space of time
        # Only change the PWM by 1 each step
        # left_dif = left_pwm - self.last_left_pwm
        # right_dif = right_pwm - self.last_right_pwm

        # self.last_left_pwm += left_dif if left_dif < MAX_PWM_STEP else MAX_PWM_STEP
        # self.last_right_pwm += right_dif if right_dif < MAX_PWM_STEP else MAX_PWM_STEP

        # print(f"pwm_esquerdo: {self.last_left_pwm}\npwm_direito: {self.last_right_pwm}")

        # Publish the speed
        # self.leftPublisher.publish(int(self.last_left_pwm))
        # self.rightPublisher.publish(int(self.last_right_pwm))
        
        ########## TESTE ##########
        self.leftPublisher.publish(168)
        self.rightPublisher.publish(168)

    def mainLoop(self):
        while not rospy.is_shutdown():
            # if keyboard.is_pressed('q'):
            #     print("Keyboard interruption")
            #     break

            self.calculateUpdate()
            self.ratePublisher.sleep()

if __name__ == "__main__":
    """ main """
    objectDR = DeadReckoningOdom()
    objectDR.mainLoop()


    # plt.show()

