import rospy
from std_msgs.msg import Int32, String
import numpy as np
import time
# import keyboard
import matplotlib.pyplot as plt
import encoder, odometry, goToGoal
import plotly.graph_objects as go
import matplotlib.pyplot as plt

# MG49 commands
SET_SPEED_RIGHT = chr(100)
SET_SPEED_LEFT = chr(101)
SET_SPEED = chr(102)
GET_LEFT_ENCODER = chr(105)
GET_RIGHT_ENCODER = chr(106)
RESET_ENCODER = chr(114)

# Frequency in Hz
UPDATE_FREQUENCY = 1

# Robot info
WHEEL_RADIUS = 0.06
WHEEL_BASE = 0.37
TICKS_PER_REVOLUTION = 980
MAX_PWM = 48
MAX_PWM_STEP = 10           # Biggest PWM step made each loop.
MAX_SPEED_DISTANCE = 1      # Distance (meters) from goal before the robot start reducing its speed.

PLOTTING = True

PATH = [[1,1]]      # An array of x and y coordinates that the robot must follow
step = 0            # Index for the actual target
goal = PATH[step]   # Actual target coordinate

class DeadReckoningOdom():
    def __init__(self):
        # TEST PATH - trajetória a ser seguida
        self.goal = goal
        self.path = PATH
        self.step = step

        # Update Frequency
        self.updateFrequencyPublish = UPDATE_FREQUENCY

        # Speed limits
        self.max_speed_distance = MAX_SPEED_DISTANCE
        self.max_pwm = MAX_PWM

        # Robot geometry
        self.wheelRadius = WHEEL_RADIUS
        self.wheelBase = WHEEL_BASE

        # Encoders
        self.left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
        self.right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)

        # Odometry
        self.odom = odometry.odometry(self.left_wheel_encoder, self.right_wheel_encoder, self.wheelBase)
        self.x = 0
        self.y = 0
        self.theta = 0

        # PID Controller
        self.controller = goToGoal.GoToGoal()
        self.w = 0      # Angular speed
        self.last_w = 0

        # Time stamps
        self.start_time = time.time_ns()
        self.last_read = time.time()

        # Published PWM
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


        ############################## ROS DEFINITION ##############################
        # ROS Node name to this class
        self.nodeName = "DeadReckoningOdom"

        # ROS Topic names
        self.topicNameLeftEncoder = "left_encoder_pulses"
        self.topicNameRightEncoder = "right_encoder_pulses"
        self.topicNamePublisher = "arduino_command"

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
        ''' Callback function called when "left_encoder_pulses" topic receive a message'''
        if message1.data:
            self.left_wheel_encoder.counter = message1.data

    def callBackFunctionRightEncoder(self, message2):
        ''' Callback function called when "right_encoder_pulses" topic receive a message'''
        if message2.data:
            self.right_wheel_encoder.counter = message2.data

    def calculateUpdate(self):
        ''' Function that executes in loop
            Reads the encoders and calculate the actual coordinates of the robot (odometry).
            Gets those coordinates and calculate the error between the robot position and the target.
            Apply the PID controll, returning a PWM value for each motor, 
            publishing it respective ROS topics.
        '''
        # Calculate how many ns passed since last read
        t = time.time_ns()
        dt = t - self.start_time
        self.start_time = t

        # Run odometry step to update robot location
        self.odom.step()
        self.x, self.y, self.theta = self.odom.getPose()

        # Calculates the angular speed w
        self.w = self.controller.step(self.goal[0], self.goal[1], self.x, self.y, self.theta, dt, precision = 0.05)
        if self.w != None: 
            self.last_w = self.w
        print(f"velocidade angular calculada: {self.w}")

        # If reach the target, the controller will return None for angular speed
        # if this is the case, consider the last calculated speed
        if self.w == None: 
            self.w = self.last_w
            # Then, check if there is a next coordinate to go in the path
            # If the path continues, makes the next point the goal
            if step+1 < len(PATH):
                step += 1
                self.goal = PATH[step]
                self.w = self.last_w
                
                # mark the new goal in the figure
                plt.scatter(self.goal[0], self.goal[1], marker='x', color='r')

            # If there is no more points in path, end the code
            else: return False

        # Convert the angular speed into differential speed for each wheel
        left_diff, right_diff = odometry.uni_to_diff(5, self.w, self.left_wheel_encoder, self.right_wheel_encoder, self.wheelBase)

        # Normalize the result speed
        if left_diff > right_diff:
            left_norm = left_diff/left_diff
            right_norm = right_diff/left_diff
        else:
            left_norm = left_diff/right_diff
            right_norm = right_diff/right_diff

        # Calculates the maximum speed based on the distance of the robot for goal
        # This makes the robot "breakes" when get close to the target
        max_speed = self.controller.speed_limit_by_distance(self.max_speed_distance, self.max_pwm, self.goal[0], self.goal[1], self.x, self.y)
        left_pwm = left_norm*max_speed
        right_pwm = right_norm*max_speed

        # The MG49 Driver reads the speed in range 0 - 255. Values greater than 128 are "positive" speeds,
        # while values between 0 and 128 are the negative ones. So, the 128 must be considered the 0 speed.
        left_pwm += 128
        right_pwm += 128

        # Take a little step in the direction of the speed calculated by the Controller
        # This is made to prevent a huge speed change in a small space of time
        # Gets the difference from the new PWM and the last one.
        delta_left = left_pwm - self.last_left_pwm
        delta_right = right_pwm - self.last_right_pwm

        # The new speed will be the last pwm + the step
        left_command = self.last_left_pwm + delta_left if delta_left < MAX_PWM_STEP else MAX_PWM_STEP
        right_command = self.last_right_pwm + delta_right if delta_right < MAX_PWM_STEP else MAX_PWM_STEP

        # print(f"pwm_esquerdo: {left_command}\npwm_direito: {right_command}")

        # Store the new PWM
        self.last_left_pwm = left_command
        self.last_right_pwm = right_command

        # Publish the speed
        self.leftPublisher.publish(int(left_command))
        self.rightPublisher.publish(int(right_command))
        
        ########## TESTE ##########
        # self.leftPublisher.publish(168)
        # self.rightPublisher.publish(168)

        # Add a point on plot figure
        if PLOTTING:
            self.pose_log['x'].append(self.x)
            self.pose_log['y'].append(self.y)
            self.pose_log['theta'].append(self.theta)

            # Add a plot
            self.line.set_offsets(np.c_[self.pose_log['x'], self.pose_log['y']])
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            print(f"x: {self.x}, y:{self.y}, t:{self.theta}")

        # End the loop, keep running the code    
        return True

    def mainLoop(self):
        running = True

        while running and not rospy.is_shutdown():
            # if keyboard.is_pressed('q'):
            #     print("Keyboard interruption")
            #     break

            running = self.calculateUpdate()
            self.ratePublisher.sleep()

if __name__ == "__main__":
    """ main """
    objectDR = DeadReckoningOdom()
    objectDR.mainLoop()

    # rospy.on_shutdown(function)

