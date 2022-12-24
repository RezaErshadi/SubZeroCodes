import serial
import pynmea2
serGarmin = serial.Serial('/dev/ttyS6',38400,timeout=1)
while True:
    a = serGarmin.readline().decode()
    print(a)
    b = pynmea2.parse(a)
    print(b)


