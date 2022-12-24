import socket
import serial
import time
import pynmea2

class GpsStreamer:

    def __init__(self,WhichGPS):
        if WhichGPS == "Trimble":
            print("Initiating Trimble")
            self.PortGPS = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.PortGPS.connect(('192.168.2.5',5017))
            print("Trimble GPS Is Initiated")
        elif WhichGPS == "Neumayer":
            print("Initiating Neumayer")
            self.PortGPS = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.PortGPS.connect(('192.168.33.97',3007))
            print("Neumayer GPS Is Initiated")
        elif WhichGPS == "Garmin":
            print("Initiating Garmin")
            self.PortGPS = serial.Serial('/dev/ttyS6',38400,timeout=1)
            print("Garmin GPS Is Initiated")
        self.gga_flag = False
        self.rmc_flag = False
        self.StreamerTimer = time.perf_counter()

    def DecimalDegree(self,x,dir):
        Deg = int(x) // 100
        Min = x - (100 * Deg)
        dd = round(Deg + Min / 60,7)
        conA = (dir == "N" and dd<0)
        conB = (dir == "S" and dd>0)
        conC = (dir == "E" and dd<0)
        conD = (dir == "W" and dd>0)
        if conA or conB or conC or conD:
            dd = -dd
        return dd

    def ParseGGA(self,sGGA,msg):
        try:
            GGAmsg = pynmea2.parse(sGGA)
            msg.msg = str(GGAmsg)
            t = str(GGAmsg.timestamp)
            tt = t.split(':')
            H = tt[0]
            M = tt[1]
            S = tt[2]
            if len(S) > 4:
                S = S[0:5]
            msg.time = f"{H}{M}{S}"
            msg.lat = self.DecimalDegree(float(GGAmsg.lat),GGAmsg.lat_dir)
            msg.lon = self.DecimalDegree(float(GGAmsg.lon),GGAmsg.lon_dir)
            msg.alt = round( (float(GGAmsg.altitude)) ,2)
            msg.geoid = float(GGAmsg.geo_sep)
            iq = GGAmsg.gps_qual
            q = ["NotAvailable", #0
                "GPSfix", #1
                "DifferentialGPS", #2
                "-",
                "RTKint", #4
                "RTKfloat"] #5
            msg.iqual = iq
            msg.qual = q[iq]
        except:
            return msg
        return msg

    def ParseRMC(self,sRMC,msg):
        try:
            RMCmsg = pynmea2.parse(sRMC)
            msg.msg = str(RMCmsg)
            t = str(RMCmsg.timestamp)
            tt = t.split(':')
            H = tt[0]
            M = tt[1]
            S = tt[2]
            if len(S) > 4:
                S = S[0:5]
            msg.time = f"{H}{M}{S}"
            msg.lat = self.DecimalDegree(float(RMCmsg.lat),RMCmsg.lat_dir)
            msg.lon = self.DecimalDegree(float(RMCmsg.lon),RMCmsg.lon_dir)
            msg.date = str(RMCmsg.datestamp)
            msg.speed = round( (float(RMCmsg.spd_over_grnd) * (0.514444)) ,2) # GPS speed in m/s
            msg.course = round( (float(RMCmsg.true_course)) ,2)
            msg.iqual = 9
            msg.qual = "RMC"
        except:
            return msg
        return msg