#!/usr/bin/env python3
"""
This node always listens to the messages from Arduino and Radio
and make important decisions.
"""

import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module
# Interfaces
from std_msgs.msg import String
from std_msgs.msg import Bool
from interfaces_subzero.msg import MyGPSmsg
from interfaces_subzero.srv import SendString
from interfaces_subzero.srv import ApRESConf
# Python packages
import datetime

class ARK(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        # ---Initiate the Node object
        super().__init__('ARK')
        # --- Initiate Parameters
        self.WhichGPS = "GARMIN"
        self.RadioPingTimer = datetime.datetime.now()
        # ---Initiate Subscriber
        self.ListenToGPS = self.create_subscription(MyGPSmsg,'GPSPub_topic',self.UpdateGPS,10)
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        # ---Initiate Clients
        self._client_Radio = self.create_client(SendString,"Radio_service")
        self._client_Trimble = self.create_client(SendString,"Trimble_service")
        self._client_Garmin = self.create_client(SendString,"Garmin_service")
        self._client_ApRES = self.create_client(ApRESConf,"ApRES_service")
        # ---Initiate Publishers
        # self.StatusPublisher = self.create_publisher(Bool,'topic_HealthARK', 10)
        # ---Initiate Timers
        # self.create_timer(1,self.timer_Status_Publisher)
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def radio_listener(self, msg):
        RadioMsg = msg.data
        comma_splt = RadioMsg.split(",")
        RadioCommand = comma_splt[0]
        RadioSubCommand = comma_splt[1:]
            #------------------------------
        if RadioCommand == "SwitchGPS":
            # "GARMIN/TRIMBLE"
            if self.WhichGPS == "Trimble":
                self.WhichGPS = "Garmin"
            else:
                self.WhichGPS = "Trimble"
            self.Client_Write2Trimble(self.WhichGPS)
            self.Client_Write2Garmin(self.WhichGPS)
            #------------------------------
        elif RadioCommand == "UpdateHome":
            # "UpdateHome"
            self.ThisIsHome = self.RoverNow
            self.get_logger().info(f"Home Updated {self.ThisIsHome}")
            MsgForRadio = f"HomeUpdated,{self.ThisIsHome[0]},{self.ThisIsHome[1]}"
            self.Client_Write2Radio(MsgForRadio)
            #------------------------------
        elif RadioCommand == "ApRES":
            if RadioSubCommand[0] != "Burst":
                self.Client_Write2ApRES(RadioSubCommand)
            #------------------------------
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    # def timer_Status_Publisher(self):
    #     msg = Bool()
    #     msg.data = True
    #     self.StatusPublisher.publish(msg)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """            
    def UpdateGPS(self,msg):   
        # update rover position
        self.RoverNow = [msg.lat,msg.lon]
        # update time from the gps
        self.gpsTime = msg.time  
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2ApRES(self,MsgForApRES):
        request = ApRESConf.Request()
        request.act = MsgForApRES[0]
        if MsgForApRES[0] == "Set":
            request.type = MsgForApRES[1]
            request.n_attenuator = int(MsgForApRES[2])
            request.attenuators = list(map(int, MsgForApRES[3:7]))
            request.gains = list(map(int, MsgForApRES[7:11]))
            request.tx = list(map(int, MsgForApRES[11:19]))
            request.rx = list(map(int, MsgForApRES[19:27]))
            request.polarization = MsgForApRES[27]
            request.n_subburst = int(MsgForApRES[28])
            request.download = False
            if MsgForApRES[29] == "true":
                request.download = True
            request.prefix = MsgForApRES[30]
        self._client_ApRES.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        request = SendString.Request()
        request.data = MsgForRadio
        self._client_Radio.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Trimble(self,GPSmode):
        request = SendString.Request()
        request.data = GPSmode
        self._client_Trimble.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Garmin(self,GPSmode):
        while not self._client_Garmin.wait_for_service(1.0):
            pass
        request = SendString.Request()
        request.data = GPSmode
        self._client_Garmin.call_async(request)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
def main(args=None):
    rclpy.init(args=args) # initialize ROS communications
    node = ARK() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
