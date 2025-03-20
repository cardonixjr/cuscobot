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
            raise Exception(f"Serial communication failed on port {self.port}: {e}")
    
    def close(self):
        self.communication.close()

    def send_data(self, data):
        self.communication.flush()
        message = str(data) + '\r'
        self.communication.write(message.encode('utf-8'))
        time.sleep(0.001)

    def get_data(self):
        line = self.communication.readline().decode('utf-8').strip()
        print(line)
        if line: 
            # print(f"python received: {line}")

            data = json.loads(line)
            return data