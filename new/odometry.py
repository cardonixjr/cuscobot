import math

class odometry():

    def __init__(self, el, er, L):
        '''
            Initialize odometry
            
            el - left wheel encoder
            er - right wheel encoder
            L - distance  between the wheels in meters
        '''
        self.el = el
        self.er = er
        self.L = L
        self.wl_last_counter = 0
        self.wr_last_counter = 0
        
        self.x = 0
        self.y = 0
        self.theta = 0
        
        # Calculate meters per tick
        self.meters_per_tick_left = (2 * math.pi * self.el.radius) / (el.ticks_p_revol)
        self.meters_per_tick_right = (2 * math.pi * self.er.radius) / (er.ticks_p_revol)

        # RPM speed
        self.wl_speed_rpm = (60/self.el.ticks_p_revol) * 10**9
        self.wr_speed_rpm = (60/self.er.ticks_p_revol) * 10**9


    def step(self, left_direction=1, right_direction=1):
        '''
            Call this function periodically to update robot pose estimiation.
        '''
        # Calculate the delta for ticks since last read
        delta_ticks_left = (self.el.counter - self.wl_last_counter) * left_direction
        delta_ticks_right = (self.er.counter - self.wr_last_counter) * right_direction
        
        # Update counters to next read
        self.wl_last_counter = self.el.counter
        self.wr_last_counter = self.er.counter
        
        # Calculate new pose
        Dl = self.meters_per_tick_left * delta_ticks_left
        Dr = self.meters_per_tick_right * delta_ticks_right
        Dc = (Dr + Dl) / 2
        
        x_dt = Dc * math.cos(self.theta)
        y_dt = Dc * math.sin(self.theta)
        theta_dt = (Dr - Dl) / self.L

        self.x = self.x + x_dt
        self.y = self.y + y_dt
        self.theta = self.theta + theta_dt
        
        return self.x, self.y, self.theta
        
    def resetPose(self):
        self.x = 0
        self.y = 0
        self.theta = 0
    
    def getPose(self):
        return self.x, self.y, self.theta
    
class speedEstimator():

    def __init__(self, el, er, R, L):
        '''
            Initialize speed estimator
            
            el - left wheel encoder
            er - right wheel encoder
            R - radious of the wheels in meters
            L - distance  between the wheels in meters
        '''
        self.el = el
        self.er = er
        self.R = R
        self.L = L
        self.wl_last_counter = 0
        self.wr_last_counter = 0
        
        # Speed converter for rpm
        self.wl_speed_rpm = (60/self.el.ticks_p_resol) * 10**9
        self.wr_speed_rpm = (60/self.er.ticks_p_resol) * 10**9

    def wheelSpeed(self, dt, left_direction=1, right_direction=1):
        '''
            Calculate wheel speeds. Call this function between 50ms to 200ms.
            
            dt - time delta since last read (in ns)
            left_direction - the direction the left motor is spinning
            right_direction = the direction the right motor is spinning
        '''
        wl_delta_ticks = self.el.counter - self.wl_last_counter
        wr_delta_ticks = self.er.counter - self.wr_last_counter

        self.wl_last_counter = self.el.counter
        self.wr_last_counter = self.er.counter

        left_wheel_speed = left_direction * wl_delta_ticks/dt * self.wl_speed_rpm
        right_wheel_speed = right_direction * wr_delta_ticks/dt * self.wr_speed_rpm

        return (left_wheel_speed, right_wheel_speed)

    def robotSpeed(self, left_rpm, right_rpm):
        left_rad_s = math.pi/30 * left_rpm
        right_rad_s = math.pi/30 * right_rpm

        v = self.R/2 * (right_rad_s + left_rad_s)
        w = self.R/self.L * (right_rad_s - left_rad_s)

        return v, w

def uni_to_diff(v,w, el, er, L):

    vel_r = (2 * v + w * L)/(2 * er.radius)
    vel_l = (2 * v - w * L)/(2 * el.radius)
    
    return vel_l, vel_r