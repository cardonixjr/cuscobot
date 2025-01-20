import time, keyboard
import matplotlib.pyplot as plt
import numpy as np
import pygame
import encoder, odometry, serialCom, goToGoal
import plotly.graph_objects as go

# Serial communication variables
PORT = "COM8"
BAUDRATE = 9600
TIMEOUT = 0.1

# Robot info
WHEEL_RADIUS = 0.06
WHEEL_BASE = 0.335
TICKS_PER_REVOLUTION = 980
MAX_PWM = 48
MAX_PWM_STEP = 10 # Maior mudança de PWM a cada loop. Isso igual a MAX_PWM é basicamente sem limitação de tamanho de passo.
MAX_SPEED_DISTANCE = 1 # Distância em metros antes do robô começar a reduzir a velocidad

# Encoders
left_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)
right_wheel_encoder = encoder.encoder(TICKS_PER_REVOLUTION, WHEEL_RADIUS)

# Open serial communication
arduino = serialCom.Communication(PORT, BAUDRATE, TIMEOUT)
arduino.open()
time.sleep(3)
# arduino.reset_encoder()
time.sleep(1)

# Set a clock to handle the loop speed
clock = pygame.time.Clock()
FPS = 100

# Odometry
odom = odometry.odometry(left_wheel_encoder, right_wheel_encoder, WHEEL_BASE)
x = 0
y = 0
theta = 0

# PID Controller
controller = goToGoal.GoToGoal()

# aux
start_time = time.time_ns()
last_read = time.time()

last_left_pwm = 128
last_right_pwm = 128

# Ploting
pose_log = {'x':[], 'y':[], 'theta':[]}

fig, ax = plt.subplots()
line = ax.scatter(pose_log['x'], pose_log['y'])

plt.axis([-2,2,-2,2])
plt.show(block=False)
plt.pause(.1)

# main loop
PATH = [[0,1],[1,1],[1,0],[0,0]]
step = 0
goal = PATH[step]
plt.scatter(goal[0], goal[1], marker='x', color='r')

end = False
while not end:
    # Check for keyboard interruption
    if keyboard.is_pressed('q'):
        print("Keyboard interruption")
        break

    # Read encoder pulses:
    le = arduino.get_encoder_left()
    ld = arduino.get_encoder_right()

    if le and ld:
        right_wheel_encoder.counter = int(ld)
        left_wheel_encoder.counter = int(le)
        last_read = time.time()
    # If can't read anything, use the last value
    # If passed a long time since the last read, stop the code
    elif time.time() - last_read > 5:
        print("muito tempo sem ler arduino, encerrando...")
        break
    # print(f"pd: {right_wheel_encoder.counter} ; pe: {left_wheel_encoder.counter}")
    
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
    # print(f"x: {x}, y:{y}, t:{theta}")

    # Calculates the angular speed w
    w = controller.step(goal[0], goal[1], x, y, theta, dt, precision = 0.05)
    if w != None: last_w = w
    if w == None: # Se chegar ao destino, vai para o próximo ponto da trajetória 
        if step+1 == len(PATH):  break  # Se chegar ao fim da trajetória, encerra o código
        step += 1
        goal = PATH[step]
        plt.scatter(goal[0], goal[1], marker='x', color='r')
        w = last_w
    print(f"velocidade angular calculada: {w}")
    
    left, right = odometry.uni_to_diff(5, w, left_wheel_encoder, right_wheel_encoder, WHEEL_BASE)
    # print(f"left: {left}\nright: {right}")

    # Normalize the result speed
    if left > right:
        left_norm = left/left
        right_norm = right/left
    else:
        left_norm = left/right
        right_norm = right/right
    # print(f"l_norm: {left_norm} ; r_norm: {right_norm}")

    max_speed = controller.speed_limit_by_distance(MAX_SPEED_DISTANCE, MAX_PWM, goal[0], goal[1], x, y)
    left_pwm = left_norm*max_speed
    right_pwm = right_norm*max_speed

    # The MG49 Driver reads the speed in range 0 - 255. Values greater than 128 are "positive" speeds,
    # while values between 0 and 128 are the negative ones. So, the 128 must be considered the speed 0.
    left_pwm += 128
    right_pwm += 128

    # Made a little step in the direction of the speed calculated by the Controller
    # This is made to prevent a huge speed change in a small space of time
    # Only change the PWM by 1 each step

    left_dif = left_pwm - last_left_pwm
    right_dif = right_pwm - last_right_pwm

    last_left_pwm += left_dif if left_dif < MAX_PWM_STEP else MAX_PWM_STEP
    last_right_pwm += right_dif if right_dif < MAX_PWM_STEP else MAX_PWM_STEP

    print(f"pwm_esquerdo: {last_left_pwm}\npwm_direito: {last_right_pwm}")
    arduino.set_speed_left(last_left_pwm)
    arduino.set_speed_right(last_right_pwm)

    # Add a plot
    line.set_offsets(np.c_[pose_log['x'], pose_log['y']])
    fig.canvas.draw()
    fig.canvas.flush_events()

    # Limit to 10 frames per second. (100ms loop)
    clock.tick(FPS)

arduino.set_speed_left(128)
arduino.set_speed_right(128)

fig = go.Figure()
fig.add_trace(go.Scatter(x=pose_log['x'], y=pose_log['y'], mode='lines+markers', name='Trajetória'))
fig.update_layout(title='Trajetória do Robô Móvel', xaxis_title='Posição X (metros)', yaxis_title='Posição Y (metros)')
# Exibe o gráfico no navegador
fig.show()

arduino.close()
plt.show()
pygame.quit()
     