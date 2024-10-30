import serial, time, math, json
#import matplotlib.pyplot as plt
import plotly.graph_objects as go

# Serial communication variables
PORT = "COM12"
BAUDRATE = 9600
TIMEOUT = 0.1

def open_communication(baudrate, port, timeout=0.1):
    # Open the serial communication
    try:
        communication = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    except serial.SerialException as e:
        print(f"Serial communication failed on port {PORT}: {e}")
        return False

    while not communication.is_open(): pass
    print(f"Serial communication sucessfully opened in port {port}, with baudrate {baudrate}")
    return communication

class OdometryCalculator:
    def __init__(self, wheel_radius=0.0835, wheel_base=0.35, ticks_per_revolution=90):
        # Parâmetros do robô
        self.wheel_radius = wheel_radius  # Raio da roda (metros)
        self.wheel_base = wheel_base      # Distância entre as rodas (metros)
        self.m_per_tick = (2 * math.pi * wheel_radius) / ticks_per_revolution

        # Estados internos
        self.prev_ticks = {'left': 0, 'right': 0}
        self.pose = [0.0, 0.0, 0.0]  # Posição estimada [x, y, theta]

    def update_odometry(self, left_ticks, right_ticks):
        """
        Atualiza a odometria baseada nos ticks dos encoders.
        """

        # Diferencas entre os ticks atuais e os anteriores
        d_right = (right_ticks - self.prev_ticks['right']) * self.m_per_tick
        d_left = (left_ticks - self.prev_ticks['left']) * self.m_per_tick

        # Calcula o deslocamento do centro do robô e a mudança de orientação
        d_center = (d_right + d_left) / 2
        phi = (d_right - d_left) / self.wheel_base

        # Atualiza a estimativa de posição
        theta_new = self.pose[2] + phi
        x_new = self.pose[0] + d_center * math.cos(self.pose[2])
        y_new = self.pose[1] + d_center * math.sin(self.pose[2])

        # Normaliza theta para o intervalo [-pi, pi]
        theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))

        # Salva os novos valores
        self.pose = [x_new, y_new, theta_new]
        self.prev_ticks = {'left': left_ticks, 'right': right_ticks}

        # Retorna a nova posição estimada
        return self.pose

odometry = OdometryCalculator()

def read_pulses():
    line = arduino.readline().decode('utf-8').strip()
    if line: 
        # print(f"python received: {line}")
        # Processa o comando vindo do arduino para pegar apenas os valores
        # Neste caso, o comando tem formato "pulsoe: 1214 pulsod: 608"

        data = json.loads(line)
        
        return data    
        

def send_serial(data):
    arduino.flush()
    message = str(data) + '\r'
    arduino.write(message.encode('utf-8'))
    time.sleep(0.01)


if __name__ == "__main__":
    arduino = open_communication(BAUDRATE, PORT, TIMEOUT)

    # cada passo é dado por (pwme,pwmd,dire,dird e tempo)
    path = [(0,0,1,1,2),(20,10,1,1,2),(0,0,1,1,2),(20,10,0,1,0.5)]
    # path = [(0,0,1,1,2),(15,7.5,1,1,2),(0,0,1,1,2),(15,7.5,0,1,1)]
    #path = [(10,5,1,1,2)] #reta
    marks = [()]

    # Listas para armazenar posições x e y
    x_positions = []
    y_positions = []


    init_time = time.time()
    last_step_time = time.time()

    time.sleep(1)

    arduino.flush()
    for i in range(4):
        for step in path:
            print(f"executando passo: {step}")
            send_serial(f"pwme,{step[0]}")
            send_serial(f"pwmd,{step[1]}")
            send_serial(f"dire,{step[2]}")
            send_serial(f"dird,{step[3]}")
            
            while not (time.time() - last_step_time > step[4]):
                data = read_pulses()
                
                if data:
                    pose = odometry.update_odometry(data["pd"], data["pe"])
                    # print(f"Posição estimada: x={pose[0]:.3f}, y={pose[1]:.3f}, theta={pose[2]:.3f}")
                    print(data)

                    # Armazena a posição (x, y) para o gráfico
                    x_positions.append(pose[0])
                    y_positions.append(pose[1])
            
            last_step_time = time.time()

    send_serial(f"pwme,0")
    send_serial(f"pwmd,0")
    send_serial(f"dire,1")
    send_serial(f"dird,1")

    #plt.figure(figsize=(8, 6))
    #plt.plot(x_positions, y_positions, marker='o', color='b', linestyle='-', label="Trajetória do robô")
    #plt.title("Trajetória do robô no plano (x, y)")
    #plt.xlabel("Posição X (metros)")
    #plt.ylabel("Posição Y (metros)")
    #plt.legend()
    #plt.grid(True)
    #plt.show()
    # Plotando a trajetória com Plotly
    fig = go.Figure()

    fig.add_trace(go.Scatter(x=x_positions, y=y_positions, mode='lines+markers', name='Trajetória'))

    fig.update_layout(title='Trajetória do Robô Móvel', xaxis_title='Posição X (metros)', yaxis_title='Posição Y (metros)')

    # Exibe o gráfico no navegador
    fig.show()