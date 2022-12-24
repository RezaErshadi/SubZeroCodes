import serial
import pynmea2
serGarmin = serial.Serial('/dev/ttyS6',38400,timeout=1)
while True:
    a = serGarmin.readline().decode()
    print(f"------{a}")
    print("//////////////////////////////")
    # b = pynmea2.parse(str(a))
    # print(b)
    # if "RMC" in str(b):
    #     print(b.status)


