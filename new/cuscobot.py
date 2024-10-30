import time, math, json, serial, pygame
import encoder, odometry, serialCom, goToGoal

# Serial communication variables
PORT = "COM3"
BAUDRATE = 9600
TIMEOUT = 0.1

# Robot info
WHEEL_RADIUS = 0.0835
WHEEL_BASE = 0.35
TICKS_PER_REVOLUTION = 90
MAX_PWM = 25

# Encoders
left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)

# Open serial communication
arduino = serialCom.Communication(PORT, BAUDRATE, TIMEOUT)
arduino.open()
time.sleep(5)

# Set a clock to handle the loop speed
clock = pygame.time.Clock()

# Odometry
odom = odometry.odometry(left_wheel_encoder, right_wheel_encoder, WHEEL_BASE)
x = 0
y = 0
theta = 0

# PID Controller
controller = goToGoal.GoToGoal()

# aux
start_time = time.time_ns()

last_left_pwm = 0
last_right_pwm = 0

# main loop
goal = [1,1]
end = False
while not end:

    # Read encoder pulses:
    pulses = arduino.get_data()
    right_wheel_encoder.counter = pulses["pd"]
    left_wheel_encoder.counter = pulses["pe"]

    # Calculate how many ns passed since last read
    t = time.time_ns()
    dt = t - start_time
    start_time = t

    # Run odometry step to update robot location
    odom.step(0, 1)
    x,y,theta = odom.getPose()

    # Calculates the angular speed w
    w = controller.step(goal[0], goal[1], x, y, theta, dt)
    
    left, right = odometry.uni_to_diff(5, w, left_wheel_encoder, right_wheel_encoder, WHEEL_BASE)

    # Normalize the result speed
    if left > right:
        left_norm = left/left
        right_norm = right/left
    else:
        left_norm = left/right
        right_norm = right/right

    left_pwm = left_norm*MAX_PWM
    right_pwm = right_norm*MAX_PWM

    # Change the direction of each wheel
    if left_pwm < 0:
        left_dir = 0
        left_pwm = -left_pwm
    else:
        left_dir = 1    
    
    if right_pwm < 0:
        right_dir = 0
        right_pwm = -right_pwm
    else:
        right_dir = 1

    # Made a little step in the direction of the speed calculated by the Controller
    # This is made to prevent a huge speed change in a small space of time
    # Only change the PWM by 1 each step
    left_dif = left_pwm - last_left_pwm
    right_dif = right_pwm - last_right_pwm

    last_left_pwm += left_dif if left_dif < 1 else 1
    last_right_pwm += right_dif if right_dif < 1 else 1

    # Limit to 10 frames per second. (100ms loop)
    clock.tick(10)

arduino.send_data(f"pwm,0")
arduino.send_data(f"dir,0")

pygame.quit()
     