#!/usr/bin/env python3
"""
This node opens a TCP IP port for R9s Trimble GPS
It always listen to the messages coming from the GPS and publishes them via:
topic_MsgFromGPS
It uses a custom msg type with necessary gps fields names
"""
import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module

from interfaces_subzero.msg import R9sGPS
from interfaces_subzero.srv import SendString
from subzero_v1.submodules.ClassNeumayerGPS import NeumayerGPS

import socket
import pynmea2
# import datetime

class nodeR9sGPS(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        super().__init__('nodeR9sGPS')
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        self.R9sFlag = False
        self.serGPS = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            self.serGPS.connect(('192.168.33.97',3007))
            self.gpsEstablished = True
            NMGPS_obj = NeumayerGPS()
        except:
            self.gpsEstablished = False
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        # ---initialize a server (activatate and deactivate the R9s)
        self.server_ = self.create_service(SendString,"service_MsgForR9s",self.GPSmode_Server)
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        self.GPSPublisher = self.create_publisher(R9sGPS,'topic_MsgFromGPS',10)
        self.create_timer(1,self.updateGPS)
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def updateGPS(self):
        msg = R9sGPS()
        msg.gps_connection = self.gpsEstablished
        msg.age = 0.1
        if self.gpsEstablished == True:
            s = self.serGPS.recv(1024)
            s.decode("utf-8")
            s = str(s)
            while True:
                i1 = s.find("$")
                i2 = s.find("\\r")
                if i1 == -1 or i2 == -1:
                    break
                gpsSubstring = s[i1:i2]
                if "BPQ" in gpsSubstring:
                    BPQmsg = pynmea2.parse(gpsSubstring)
                    BPQquality = BPQmsg.gps_qual
                    if BPQquality > 0:
                        msg.bpq_msg = str(BPQmsg)
                        msg.bpq_time = str(BPQmsg.timestamp)
                        msg.base_lat = self.DecimalDegree(float(BPQmsg.lat),BPQmsg.lat_dir)
                        msg.base_lon = self.DecimalDegree(float(BPQmsg.lon),BPQmsg.lon_dir)
                        EHT = BPQmsg.height
                        EHT = EHT.replace('EHT','')
                        msg.base_eht = float(EHT)
                elif "GGA" in gpsSubstring:
                    GGAmsg = pynmea2.parse(gpsSubstring)
                    GGAquality = GGAmsg.gps_qual
                    if GGAquality > 0:
                        msg.gps_data = True
                        msg.gga_msg = str(GGAmsg)
                        msg.gga_time = str(GGAmsg.timestamp)
                        msg.rover_lat = self.DecimalDegree(float(GGAmsg.lat),GGAmsg.lat_dir)
                        msg.rover_lon = self.DecimalDegree(float(GGAmsg.lon),GGAmsg.lon_dir)
                        msg.rover_alt = float(GGAmsg.altitude)
                        msg.rover_geoid = float(GGAmsg.geo_sep)
                elif "RMC" in gpsSubstring:   
                    RMCmsg = pynmea2.parse(gpsSubstring)
                    RMCstatus = RMCmsg.status
                    if RMCstatus == 'A':
                        msg.rmc_msg = str(RMCmsg)
                        msg.rmc_time = str(RMCmsg.timestamp)
                        msg.rover_date = str(RMCmsg.datestamp)
                        msg.rover_speed = float(RMCmsg.spd_over_grnd) * (0.514444) # GPS speed in m/s
                        msg.rover_course = float(RMCmsg.true_course)
                s = s[i2+1:]
        msg.gps_for_arduino = f"CMDGPS,{msg.age},{msg.gga_time},{msg.rover_lat},{msg.rover_lon },{msg.rover_alt},{msg.rover_speed},{msg.rover_course}"
        if msg.gps_data == True:
            if self.R9sFlag == True:
                self.GPSPublisher.publish(msg)
                # self.get_logger().info(f"{msg.gga_msg}")
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def DecimalDegree(self,x,dir):
        Deg = int(x) // 100
        Min = x - (100 * Deg)
        dd = Deg + Min / 60
        conA = (dir == "N" and dd<0)
        conB = (dir == "S" and dd>0)
        conC = (dir == "E" and dd<0)
        conD = (dir == "W" and dd>0)
        if conA or conB or conC or conD:
            dd = -dd
        return dd 
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def GPSmode_Server(self, request, response):
        if request.data == "TRIMBLE":
            self.R9sFlag = True
            self.get_logger().info(f"TRIMBLE is Activated (GARMIN off)")
        elif request.data == "GARMIN":
            self.R9sFlag = False
            self.get_logger().info(f"GARMIN is Activated (R9sGPS off)")
        response.msg_sent = True
        return response
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
def main(args=None):
    rclpy.init(args=args) # initialize ROS communications
    node = nodeR9sGPS() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    node.destroy_node()
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()