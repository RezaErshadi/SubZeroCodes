#!/usr/bin/env python3
"""
This node opens a TCP IP port for R9s Trimble GPS
It always listen to the messages coming from the GPS and publishes them via:
topic_MsgFromTrimble
It uses a custom msg type with necessary gps fields names
"""
import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module
# Interfaces
from std_msgs.msg import String
from interfaces_subzero.msg import MyGPSmsg
from interfaces_subzero.srv import SendString
from interfaces_subzero.srv import LoggerString
# Submodules
from subzero_dec22.submodules.ClassGpsStreamer import GpsStreamer
# Python packages
import time
class Trimble(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        # ---Initiate the Node object
        super().__init__('Trimble')
        self.client_Logger = self.create_client(LoggerString,"Logger_service")
        self.Client_Write2Logger("info","Trimble: started")
        # Initiate Parameters
        self.StreamFlag = "Off"
        self.WhichGPS = "Trimble"
        self.BaudRate = 1024
        # ---Initiate GPS object
        self.Client_Write2Logger("info",f"Trimble: streaming --> {self.StreamFlag}")
        while True:
            try:
                self.Client_Write2Logger("info","Trimble: creating a GPS object")
                self.GPS_obj = GpsStreamer(self.WhichGPS)
                self.gpsEstablished = True
                self.get_logger().info("Trimble is Active")
                self.Client_Write2Logger("info","Trimble: GPS object created")
                break
            except Exception as e:
                self.StreamFlag = "Off"
                self.gpsEstablished = False
                self.Client_Write2Logger("error","Trimble: connection failed")
                self.Client_Write2Logger("exception",str(e))
                time.sleep(2)
        # ---Initiate Subscribers
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        # ---Initialize Services
        
        # ---Initialize Clients
        self.client_Radio = self.create_client(SendString,"Radio_service")
        # ---Initialize Publishers
        self.Trimble_Publisher = self.create_publisher(MyGPSmsg,'GPSPub_topic',10)
        # ---Initialize Timers
        self.create_timer(0.05,self.updateGPS)
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def updateGPS(self):
        try:
            ggaFlag = False
            rmcFlag = False
            s = self.GPS_obj.PortGPS.recv(1024)
            s = str(s.decode())
            if self.StreamFlag == "On":
                # self.get_logger().info(f"{s}")
                msg = MyGPSmsg()
                if self.WhichGPS == "Trimble":
                    msg.gps = "Trimble"
                    if "GGA" in s:
                        ggaFlag = True
                        msg.nmea = "GGA"
                        msg = GpsStreamer.ParseGGA(self.GPS_obj,s,msg)
                    elif "RMC" in s:
                        rmcFlag = True
                        msg.nmea = "RMC"
                        msg = GpsStreamer.ParseRMC(self.GPS_obj,s,msg)
                    if ggaFlag == True or rmcFlag == True:
                        self.Trimble_Publisher.publish(msg)
                elif self.WhichGPS == "Neumayer":
                    a = s.split("\r\n")
                    for i in range(len(a)-1):
                        ss = a[i]
                        msg.gps = "Neumayer"
                        if "GGA" in ss:
                            ggaFlag = True
                            msg.nmea = "GGA"
                            msg = GpsStreamer.ParseGGA(self.GPS_obj,ss,msg)
                        elif "RMC" in ss:
                            rmcFlag = True
                            msg.nmea = "RMC"
                            msg = GpsStreamer.ParseRMC(self.GPS_obj,ss,msg)
                        if ggaFlag == True or rmcFlag == True:
                            self.Trimble_Publisher.publish(msg)
        except Exception as e:
            self.Client_Write2Logger("error","Trimble: updateGPS")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def radio_listener(self, msg):
        try:
            RadioMsg = msg.data
            comma_splt = RadioMsg.split(",")
            RadioCommand = comma_splt[0]
            #------------------------------
            if RadioCommand == "SwitchGPS":
                self.SwitchGPS()
        except Exception as e:
            self.Client_Write2Logger("error","Trimble: radio_listener")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def SwitchGPS(self):
        if self.StreamFlag == "On":
            self.StreamFlag = "Off"
            self.Client_Write2Logger("info",f"Trimble: switch GPS --> turn {self.StreamFlag} streaming")
        elif self.StreamFlag == "Off":
            self.StreamFlag = "On"
            time.sleep(0.5)
            self.Client_Write2Radio("Trimble is Active")
            self.Client_Write2Logger("info","Trimble: GPS is switched to Trimble")
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        try:
            request = SendString.Request()
            request.data = MsgForRadio
            self.client_Radio.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","Trimble: Client_Write2Radio")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Logger(self,LoggerType,LoggerMsg):
        request = LoggerString.Request()
        request.type = LoggerType
        request.data = LoggerMsg
        self.client_Logger.call_async(request)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
def main(args=None):
    rclpy.init(args=args) # initialize ROS communications
    node = Trimble() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    node.destroy_node()
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()