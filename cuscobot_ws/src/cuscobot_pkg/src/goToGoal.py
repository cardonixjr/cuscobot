import math
import odometry

class GoToGoal:
    '''
        Class that define a closed loop Proportional Integrative and Derivative Control

        The PID controler receives the actual value of robot coordinates (x, y) and orientation (theta)
        and the goal coordinates (x_goal, y_goal) and calculates the angular velocity to corrects the
        trajectory to move the robot closer to his goal.

        Also have auxiliary functions to limit robot speed by the distance from the target and to
        convert the angular velocity (w) into PWM values for both MD49 motors
    '''
    def __init__(self):
        self.E_d = 0        # Error for the derivative term
        self.E_i = 0        # Error for the integrative term

        # PID gains
        #self.Kp = 100
        #self.Ki = 2
        #self.Kd = 20

        self.Kp = 100
        self.Ki = 2
        self.Kd = 20

    def step(self, x_goal, y_goal, x, y, theta, dt, precision = 0.05):
        dt = dt/1000000000

        # Distance between goal and robot position in X
        u_x = x_goal - x

        # Distance between goal and robot position in Y
        u_y = y_goal - y

        if abs(u_x) < precision and abs(u_y) < precision:
            print("voce chegou ao seu destino")
            return None
        
        # Angle from robot to goal
        theta_goal = math.atan2(u_y, u_x)

        # Error between the goal angle and robot's angle
        e_k = theta_goal - theta
        e_k = math.atan2(math.sin(e_k), math.cos(e_k))

        # Error for the proportional term
        e_P = e_k

        # Error for the integral term
        e_I = self.E_i + e_k*dt

        # Error for the derivative term
        e_D = (e_k - self.E_d)/dt

        w = self.Kp * e_P + self.Ki * e_I + self.Kd * e_D
        # print("Distance from goal", '{:02.3f}'.format(u_x), '{:02.3f}'.format(u_y))
        # print("w: ", w)

        # Save errors for next time step
        self.E_i = e_I
        self.E_d = e_k

        return w
    
    def speed_limit_by_distance(self, max_distance, max_pwm, x_goal, y_goal, x, y):
        # Distance between goal and robot position in X
        u_x = x_goal - x

        # Distance between goal and robot position in Y
        u_y = y_goal - y

        euclidean_u = math.sqrt(u_y**2 + u_x**2)

        pwm = euclidean_u*100/max_distance
        
        return pwm if pwm <= max_pwm else max_pwm

    def w_to_pwm(self, w, left_wheel_encoder, right_wheel_encoder, wheelBase, max_speed_distance, max_pwm, goal, x, y):
        x_goal = goal[0]
        y_goal = goal[1]

        # Convert the angular speed into differential speed for each wheel
        left_diff, right_diff = odometry.uni_to_diff(5, w, left_wheel_encoder, right_wheel_encoder, wheelBase)
        # print(f"left_diff = {left_diff} , right_diff = {right_diff}")

        # Normalize the result speed
        if left_diff > right_diff:
            left_norm = left_diff/left_diff
            right_norm = right_diff/left_diff
        else:
            left_norm = left_diff/right_diff
            right_norm = right_diff/right_diff
        # print(f"norm_left = {left_norm},  norm_right = {right_norm}")

        # Calculates the maximum speed based on the distance of the robot for goal
        # This makes the robot "breakes" when get close to the target
        max_speed = self.speed_limit_by_distance(max_speed_distance, max_pwm, x_goal, y_goal, x, y)
        left_pwm = left_norm*max_speed
        right_pwm = right_norm*max_speed

        # The MG49 Driver reads the speed in range 0 - 255. Values greater than 128 are "positive" speeds,
        # while values between 0 and 128 are the negative ones. So, the 128 must be considered the 0 speed.
        left_pwm += 128
        right_pwm += 128

        # Made a little step in the direction of the speed calculated by the Controller
        # This is made to prevent a huge speed change in a small space of time
        # Only change the PWM by 1 each step
        #delta_left = left_pwm - self.last_left_pwm
        #delta_right = right_pwm - self.last_left_pwm
        #left_pwm = self.last_left_pwm + delta_left if delta_left < MAX_PWM_STEP else self.last_left_pwm + MAX_PWM_STEP
        #right_pwm = self.last_right_pwm + delta_right if delta_right < MAX_PWM_STEP else self.last_right_pwm + MAX_PWM_STEP

        return left_pwm, right_pwm



def limit(value, downLimit, upLimit):
        return upLimit if value >= upLimit else downLimit if value <= downLimit else value

def rateLimit(value, ctrlVar, upLimit, downLimit):
        ctrlVar = limit(value, (downLimit + ctrlVar),(upLimit + ctrlVar))
        return ctrlVar