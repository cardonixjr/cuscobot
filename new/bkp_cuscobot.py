import time, math, json, serial, pygame
import encoder, odometry, stateControl, serialCom

# Serial communication variables
PORT = "COM3"
BAUDRATE = 9600
TIMEOUT = 0.1

# Robot info
WHEEL_RADIUS = 0.0835
WHEEL_BASE = 0.35
TICKS_PER_REVOLUTION = 90

# Encoders
left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)

# Open serial communication
arduino = serialCom.Communication(PORT, BAUDRATE, TIMEOUT)
arduino.open()
time.sleep(1)

# Set a clock to handle the loop speed
clock = pygame.time.Clock()

# Odometry
odom = odometry.odometry(left_wheel_encoder, right_wheel_encoder, WHEEL_BASE)
x = 0
y = 0
theta = 0

last_left_dir = 1
last_right_dir = 1

# State machine
st_control = stateControl.stateControl()

# main loop
start_time = time.time_ns()

end = False
while not end:
    # left, right = joy.tick()
    # left = left if left >= 0 else 0
    # right = right if right >=0 else 0

    # Read encoder pulses:
    pulses = arduino.get_data()
    right_wheel_encoder.counter = pulses["pd"]
    left_wheel_encoder.counter = pulses["pe"]

    # Calculate how many ns passed since last read
    t = time.time_ns()
    dt = t - start_time
    start_time = t

    # Run odometry step to update robot location
    odom.step(last_left_dir, last_right_dir)
    x,y,theta = odom.getPose()

    # # Check if pose needs to be reseted by the joystick
    # if (joy.resetPose == 1):
    #    print("Pose reseted")
    #    odo.resetPose()
    #    left_wheel_encoder.reset()
    #    right_wheel_encoder.reset()

    # Set inputs fot the state machine
    st_control.input.x = x
    st_control.input.y = y
    st_control.input.theta = theta
    st_control.input.dt = dt
    st_control.input.el = left_wheel_encoder
    st_control.input.er = right_wheel_encoder
    st_control.input.L = WHEEL_BASE

    # Run state machine
    st_control.step()

    #print('test2', stateControl.State.output.left_motor, stateControl.State.output.right_motor)
    # Update outputs
    # kit.motor1.throttle = stControl.output.left_motor
    # kit.motor4.throttle = stControl.output.right_motor
    pwm_a = st_control.output.left_motor
    pwm_b = st_control.output.right_motor

    # # The encoder can not know the direction of the motor, so we are
    # # going to use the motor commands to know what direction is turning
    # last_left_dir = 1 if left >=0 else -1
    # last_right_dir = 1 if right >=0  else -1
    
    # Print some info
    #print(stateControl.State.output.left_motor, stateControl.State.output.right_motor)
    # theta_d = (theta - (2 * math.pi * math.floor((theta + math.pi)/(2*math.pi))))
    # theta_d = theta * 180/math.pi
    #print('Delta time', (dt/1000000), 'x', '{:02.2f}'.format(x), 'y', '{:02.2f}'.format(y), 'theta', '{:02.2f}'.format(theta_d))
    # print('L/R:', left_wheel_encoder.counter, right_wheel_encoder.counter,
    # 'x:', '{:02.3f}'.format(x), 'y:', '{:02.3f}'.format(y), 'theta:', '{:02.2f}'.format(theta_d))

    # End loop
    # events = pygame.event.get()
    # for event in events:
    #     if event.type == pygame.KEYDOWN:
    #         if event.key == pygame.K_LEFT:
    #             end = True

    # Limit to 10 frames per second. (100ms loop)
    clock.tick(10)

arduino.send_data(f"pwm,0")
arduino.send_data(f"dir,0")

pygame.quit()
     