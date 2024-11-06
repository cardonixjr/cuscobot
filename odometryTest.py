import time, keyboard
import matplotlib.pyplot as plt
import numpy as np
import pygame
import encoder, odometry, serialCom, goToGoal
import plotly.graph_objects as go

# Serial communication variables
PORT = "COM5"
BAUDRATE = 9600
TIMEOUT = 0.1

# Robot info
WHEEL_RADIUS = 0.0835
WHEEL_BASE = 0.465
TICKS_PER_REVOLUTION = 90
MAX_PWM = 15

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

# aux
start_time = time.time_ns()
last_read = time.time()

last_left_pwm = 0
last_right_pwm = 0

arduino.send_data(f"dir,0")
arduino.send_data(f"pwm,10")

# Ploting
pose_log = {'x':[], 'y':[], 'theta':[]}

fig, ax = plt.subplots()
line = ax.scatter(pose_log['x'], pose_log['y'])

plt.axis([-2,2,-2,2])
plt.show(block=False)
plt.pause(.1)

# main loop
while 1:
    # Check for keyboard interruption
    if keyboard.is_pressed('q'):
        print("Keyboard interruption")
        break

    # Read encoder pulses:
    pulses = arduino.get_data()

    if pulses != None:
        right_wheel_encoder.counter = pulses["pd"]
        left_wheel_encoder.counter = pulses["pe"]
        last_read = time.time()
    # If can't read anything, use the last value
    # If passed a long time since the last read, stop the code
    elif time.time() - last_read > 5:
        print("muito tempo sem ler arduino, encerrando...")
        break

    # Calculate how many ns passed since last read
    t = time.time_ns()
    dt = t - start_time
    start_time = t

    # Run odometry step to update robot location
    odom.step()
    x,y,theta = odom.getPose()
    pose_log['x'].append(x)
    pose_log['y'].append(y)
    pose_log['theta'].append(theta)

    # Add a plot
    # plt.clf()
    line.set_offsets(np.c_[pose_log['x'], pose_log['y']])

    # Plot theta direction
    # plt.plot([x,y], [x+np.cos(theta), y+np.sin(theta)], marker = 'x')

    fig.canvas.draw()
    fig.canvas.flush_events()

    # Limit to 10 frames per second. (100ms loop)
    clock.tick(10)

arduino.send_data(f"pwm,0")
arduino.send_data(f"dir,0")

fig = go.Figure()
fig.add_trace(go.Scatter(x=pose_log['x'], y=pose_log['y'], mode='lines+markers', name='Trajetória'))
fig.update_layout(title='Trajetória do Robô Móvel', xaxis_title='Posição X (metros)', yaxis_title='Posição Y (metros)')
# Exibe o gráfico no navegador
fig.show()

plt.show()
pygame.quit()
     