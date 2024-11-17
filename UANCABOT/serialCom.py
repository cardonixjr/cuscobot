import serial, time

SET_SPEED_RIGHT = 100
SET_SPEED_LEFT = 101
SET_SPEED = 102
GET_LEFT_ENCODER = 105
GET_RIGHT_ENCODER = 106

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

    def set_speed_left(self, value):
        self.send_data(f"{SET_SPEED_LEFT},{value}")

        response = self.communication.readline()
        # print(response)
        return response

    def set_speed_right(self, value):
        self.send_data(f"{SET_SPEED_RIGHT},{value}")
           
        response = self.communication.readline()
        # print(response)
        return response
    
    def set_speed(self, value):
        self.send_data(f"{SET_SPEED},{value}")

        response = self.communication.readline()
        # print(response)
        return response

    def get_encoder_left(self):
        self.send_data(f"{GET_LEFT_ENCODER}")

        
        response = self.communication.readline()
        # print(response)
        return response

    def get_encoder_right(self):
        self.send_data(f"{GET_RIGHT_ENCODER}")
        
        response = self.communication.readline()
        # print(response)
        return response