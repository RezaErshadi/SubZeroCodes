import serial

serArduino = serial.Serial('/dev/ttyACM0',38400,timeout=0.1)

while True:
        if serArduino.in_waiting > 0:
            serArduino.flush()
            print("--------------------------------")
            print(f"available bytes {serArduino.in_waiting}")
            print( f"test: {serArduino.readline().decode('utf-8')}" )
            print(f"available bytes {serArduino.in_waiting}")