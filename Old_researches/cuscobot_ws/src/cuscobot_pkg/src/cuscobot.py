import rospy
from std_msgs.msg import Int32, Empty
import numpy as np
import time
#import keyboard
import matplotlib.pyplot as plt
import encoder, odometry, goToGoal
import plotly.graph_objects as go
import matplotlib.pyplot as plt
from pynput import keyboard

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
MAX_PWM = 40
MAX_PWM_STEP = 30           # Biggest PWM step made each loop.
MAX_SPEED_DISTANCE = 1      # Distance (meters) from goal before the robot start reducing its speed.

PLOTTING = True

PATH = [[1,1],[0,0]]      # An array of x and y coordinates that the robot must follow
step = 0            # Index for the actual target
goal = PATH[step]   # Actual target coordinate

#def on_release(key): return key
#def on_press(key): return key

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

        # aux
        self.is_first_loop = True

        # Plotting
        if PLOTTING:
            self.pose_log = {'x':[], 'y':[], 'theta':[]}
            self.fig, self.ax = plt.subplots()
            self.line = self.ax.scatter(self.pose_log['x'], self.pose_log['y'])

            plt.axis([-2,2,-2,2])
            plt.scatter(self.goal[0], self.goal[1], marker='x', color='r')
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

        # Puvlishers
        self.leftPublisher = rospy.Publisher("left_wheel_pwm", Int32, queue_size=5)
        self.rightPublisher = rospy.Publisher("right_wheel_pwm", Int32, queue_size=5)
        self.resetPublisher = rospy.Publisher("reset_encoder", Empty, queue_size = 1)

        # Rate publisher
        self.ratePublisher = rospy.Rate(self.updateFrequencyPublish)

        # Reset encoders
        #self.resetEncoder()

    def callBackFunctionLeftEncoder(self, message1):
        ''' Callback function called when "left_encoder_pulses" topic receive a message'''
        if message1.data:
            self.left_wheel_encoder.counter = message1.data

    def callBackFunctionRightEncoder(self, message2):
        ''' Callback function called when "right_encoder_pulses" topic receive a message'''
        if message2.data:
            self.right_wheel_encoder.counter = message2.data

    def stop(self):
        self.leftPublisher.publish(128)
        self.rightPublisher.publish(128)

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

        ############################## ODOMETRY ##############################
        self.odom.step()
        self.x, self.y, self.theta = self.odom.getPose()
        print(f"x: {self.x}, y:{self.y}, t:{self.theta}")

        ############################## PID ##############################
        # Calculates the angular speed w
        self.w = self.controller.step(self.goal[0], self.goal[1], self.x, self.y, self.theta, dt, precision = 0.05)
        if self.w != None: 
            self.last_w = self.w

        # Check if reached the target
        # If reach the target, the controller will return None for angular speed
        # if this is the case, consider the last calculated speed
        if self.w == None: 
            self.w = self.last_w
            
            # Then, check if there is a next coordinate to go in the path
            # If the path continues, makes the next point the goal
            if self.step+1 < len(self.path):
                self.step += 1
                self.goal = self.path[self.step]
                self.w = self.last_w
                
                # mark the new goal in the figure
                plt.scatter(self.goal[0], self.goal[1], marker='x', color='r')
            # If there is no more points in path, end the code
            else: return False

        left_pwm, right_pwm = self.controller.w_to_pwm(self.w, self.left_wheel_encoder, self.right_wheel_encoder, self.wheelBase, 
                                                       self.max_speed_distance, self.max_pwm, self.goal, self.x, self.y)

        ############################## ROS PUBLISH ##############################
        # Store the new PWM value
        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm

        self.leftPublisher.publish(int(left_pwm))
        self.rightPublisher.publish(int(right_pwm))

        ############################## PLOT ##############################
        if PLOTTING:
            self.pose_log['x'].append(self.x)
            self.pose_log['y'].append(self.y)
            self.pose_log['theta'].append(self.theta)

            # Add a plot
            self.line.set_offsets(np.c_[self.pose_log['x'], self.pose_log['y']])
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        # End the loop, keep running the code    
        return True

    def mainLoop(self):
        try:
            running = True

            with keyboard.Events() as events:
                while running and not rospy.is_shutdown():
                    running = self.calculateUpdate()
                    self.ratePublisher.sleep()

                    while True:
                        event = events.get(0.001)
                        if event != None:
                            print(type(event.key))
                            if event.key.char == 'q':
                                running = False
                        if event == None: break
        except Exception as e:
            print(e)

        finally:
            # Stop the robot
            self.resetPublisher.publish()
            self.stop()

            # Plot
            fig = go.Figure()
            fig.add_trace(go.Scatter(x=self.pose_log['x'], y=self.pose_log['y'], mode='lines+markers', name='Trajetória'))

            for point in PATH:
                fig.add_scatter(x=[point[0]],
                    y=[point[1]],
                    marker=dict(
                        color='red',
                        size=10,
                        symbol="x-thin"
                    ))
            fig.update_layout(title='Trajetória do Robô Móvel', xaxis_title='Posição X (metros)', yaxis_title='Posição Y (metros)')
            # Exibe o gráfico no navegador
            fig.show()


if __name__ == "__main__":
    """ main """
    objectDR = DeadReckoningOdom()
    objectDR.mainLoop()
    objectDR.stop()

    # rospy.on_shutdown(function)

