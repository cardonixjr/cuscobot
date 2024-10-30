import serial, time, json

class Communication:
    def __init__(self, port, baudrate, timeout=0.1):
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout

    def open(self):
        try:
            self.communication = serial.Serial(port=self.port, 
                                               baudrate=self.baudrate, 
                                               timeout=self.timeout)
            while not(self.communication.is_open): pass
            print(f"Serial communication successfully opened on port {self.port}")
        except serial.SerialException as e:
            print(f"Serial communication failed on port {self.port}: {e}")
        return False
    
    def close(self):
        self.communication.close()

    def send_data(self, data):
        self.communication.flush()
        message = str(data) + '\r'
        self.communication.write(message.encode('utf-8'))
        time.sleep(0.01)

    def get_data(self):
        line = self.communication.readline().decode('utf-8').strip()
        print(line)
        if line: 
            # print(f"python received: {line}")

            data = json.loads(line)
            return data

# def open_communication(baudrate, port, timeout=0.1):
#     # Open the serial communication
#     try:
#         communication = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
#     except serial.SerialException as e:
#         print(f"Serial communication failed on port {port}: {e}")
#         return False

#     while not communication.is_open(): pass
#     print(f"Serial communication sucessfully opened in port {port}, with baudrate {baudrate}")
#     return communication

# def send_serial(communication, data):
#     communication.flush()
#     message = str(data) + '\r'
#     communication.write(message.encode('utf-8'))
#     time.sleep(0.01)

# def read_pulses(communication):
#     line = communication.readline().decode('utf-8').strip()
#     if line: 
#         # print(f"python received: {line}")
#         # Processa o comando vindo do arduino para pegar apenas os valores
#         # Neste caso, o comando tem formato "pulsoe: 1214 pulsod: 608"

#         data = json.loads(line)
        
#         return data