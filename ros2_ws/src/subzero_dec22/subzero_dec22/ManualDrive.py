#!/usr/bin/env python3
"""
This node opens a serial port for Radio communication
It always listen to the messages coming from the radio and publishes them via:
topic_MsgFromRadio
in a form of comma seperated messages and main command and sub commands
It also listens to any messages from other nodes which must be sent into the radio via:
topic_MsgToRadio
"""
import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module
# Interfaces
from std_msgs.msg import String
from interfaces_subzero.srv import SendString
from interfaces_subzero.srv import SpeedValues
from interfaces_subzero.srv import LoggerString
# Python packages
import time
import subprocess
# Submodules

""" -------------------------------------------------- """
class ManualDrive(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('ManualDrive')
        self.client_Logger = self.create_client(LoggerString,"Logger_service")
        self.Client_Write2Logger("info","ManualDrive: started")
        self.DriveMode  = "Manual"
        self.V = 0.0
        self.W = 0.0
        # ---Initiate Parameters

        # ---Initiate Subscribers
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        # ---Initiate Services
        self.server_manualdrive = self.create_service(SendString,"ManualDrive_service",self.ManualDrive_Server)
        # ---Initiate Clients
        self.client_Radio = self.create_client(SendString,"Radio_service")
        self.client_WriteSpeed = self.create_client(SpeedValues,"WriteSpeed_service")
        # ---Initiate Timers

    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def radio_listener(self, msg):
        try:
            RadioMsg = msg.data
            RadioList = RadioMsg.split(",")
            RadioCommand = RadioList[0]
            RadioSubCommand = RadioList[1:]
            if RadioCommand == "CMDDriveMode":
                self.DriveMode = RadioSubCommand[0]
                self.Client_Write2Logger("info",f"ManualDrive: driving mode set to {self.DriveMode}")
            elif RadioCommand == "CMDEmStop":
                self.DriveMode  = "Manual"
                self.Client_Write2Logger("info",f"ManualDrive: driving mode set to {self.DriveMode}")
            elif RadioCommand == "ComeHome":
                self.DriveMode = "Autonomous"
                self.Client_Write2Logger("info",f"ManualDrive: driving mode set to {self.DriveMode}") 
            elif self.DriveMode  == "Manual":
                if RadioCommand == "CMDSetSpeed":
                    self.V = float(RadioSubCommand[0])
                    self.W = float(RadioSubCommand[1])
                    self.Client_WriteSpeed(self.V,self.W)
                    self.Client_Write2Logger("info",f"ManualDrive: new desired speed V:{self.V}, W:{self.W}")
                elif RadioCommand == "CMDStop":
                    self.Stop()
                    self.Client_Write2Logger("info",f"ManualDrive: Stop")
        except Exception as e:
            self.Client_Write2Logger("error","ManualDrive: radio_listener")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Stop(self):
        try:
            self.Client_WriteSpeed(0.0,0.0)
        except Exception as e:
            self.Client_Write2Logger("error","ManualDrive: Stop")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """  
    def ManualDrive_Server(self, request, response):
        try:
            msg = request.data
            if msg == "No Radio Feedback":
                if self.DriveMode == "Manual":
                    self.Client_Write2Logger("warning","PingPongRadio: no radio feedback (pong)")
                    if self.V != 0.0:
                        self.Client_Write2Logger("warning","ManualDrive: stopping the rover")
                        self.Stop()
            response.msg_sent = True
            return response
        except Exception as e:
            self.Client_Write2Logger("error","ManualDrive: ManualDrive_Server")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        try:
            request = SendString.Request()
            request.data = MsgForRadio
            self.client_Radio.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","ManualDrive: Client_Write2Radio")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_WriteSpeed(self,V,W):
        try:
            request = SpeedValues.Request()
            request.v_linear = float(V)
            request.w_angular = float(W)
            self.client_WriteSpeed.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","ManualDrive: Client_WriteSpeed")
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
    node = ManualDrive() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
