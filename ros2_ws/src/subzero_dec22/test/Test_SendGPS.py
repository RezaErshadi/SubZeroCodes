import socket
import pynmea2
import serial

def DecimalDegree(x):
    Deg = int(x) // 100
    Min = x - (100 * Deg)
    dd = Deg + Min / 60 
    return dd 
radSer = serial.Serial('/dev/ttyUSB0',38400,timeout=1)
gpsPort = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
gpsPort.connect(('192.168.33.97',3007))
rover_date = ""
gga_time = ""
rover_lat = ""
rover_lon = ""
rover_alt = ""
rover_speed = ""
rover_course = ""
while True:
    s = gpsPort.recv(1024)
    s.decode("utf-8")
    s = str(s)
    rover_lat  = -99999.0
    while True:
        i1 = s.find("$")
        i2 = s.find("\\r")
        if i1 == -1 or i2 == -1:
            break
        gpsSubstring = s[i1:i2]
        if "BPQ" in gpsSubstring:
            BPQmsg = pynmea2.parse(gpsSubstring)
            bpq_msg = str(BPQmsg)
            bpq_time = str(BPQmsg.BPQtimestamp)
            base_lat = DecimalDegree(float(BPQmsg.lat))
            base_lon = DecimalDegree(float(BPQmsg.lon))
            EHT = BPQmsg.height
            EHT = EHT.replace('EHT','')
            base_eht = float(EHT)
        elif "GGA" in gpsSubstring:
            GGAmsg = pynmea2.parse(gpsSubstring)
            # print(GGAmsg.data)
            # print(GGAmsg.fields)
            # print(GGAmsg)
            # print(type(GGAmsg))
            # print(f"{GGAmsg.timestamp} , {type(GGAmsg.timestamp)}")
            # print(f"{GGAmsg.lat} , {type(GGAmsg.lat)}")
            # print(f"{GGAmsg.altitude} , {type(GGAmsg.altitude)}")
            # print(f"{GGAmsg.geo_sep} , {type(GGAmsg.geo_sep)}")
            gga_msg = str(GGAmsg)
            gga_time = str(GGAmsg.timestamp)
            rover_lat = DecimalDegree(float(GGAmsg.lat))
            rover_lon = DecimalDegree(float(GGAmsg.lon))
            rover_alt = float(GGAmsg.altitude)
            rover_geoid = float(GGAmsg.geo_sep)
        elif "RMC" in gpsSubstring:   
            RMCmsg = pynmea2.parse(gpsSubstring)
            print(RMCmsg.data)
            print(RMCmsg.fields)
            rmc_msg = str(RMCmsg)
            rmc_time = str(RMCmsg.timestamp)
            rover_date = str(RMCmsg.datestamp)
            rover_speed = float(RMCmsg.spd_over_grnd) * (0.514444) # GPS speed in m/s
            rover_course = float(RMCmsg.true_course)
        s = s[i2+1:]

    # gps_for_cs = f"#{rover_date},{gga_time},{rover_lat},{rover_lon },{rover_alt},{rover_speed},{rover_course}$\r\n"
    # radSer.write(bytes(gps_for_cs,'utf-8'))
    # gps_for_cs = f"#TELLGPS,{gga_time},{rover_lat},{rover_lon },{rover_alt},{rover_speed},{rover_course}$\r\n"
    # print(gps_for_cs)





