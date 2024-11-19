import serialCom, time

# Open serial communication
arduino = serialCom.Communication("COM8", 9600, 0.1)
arduino.open()
time.sleep(5)

path = [[138,138],[118,118],[138,118],[118,138],[128,128]]
step = 0
init_time = time.time()

while 1:
    # pe = arduino.get_encoder_left()
    # pd = arduino.get_encoder_right()
    pe, pd = arduino.get_encoders()
    print(f"pe: {pe} ; pd: {pd}")
    time.sleep(1)

    if time.time() - init_time > 6:
        if step + 1 >= len(path):
            break
        step += 1
        init_time = time.time()
        print("mudou a velocidade")

    arduino.set_speed_left(path[step][0])
    arduino.set_speed_right(path[step][1])

arduino.close()