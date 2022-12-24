import socket

PortGPS = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
PortGPS.connect(('192.168.1.5',5017))
while True:
    s = PortGPS.recv(1024)
    s.decode("utf-8")
    print(s)