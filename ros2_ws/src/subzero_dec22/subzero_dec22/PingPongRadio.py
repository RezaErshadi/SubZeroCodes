#!/usr/bin/env python3
"""

"""
import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module
# Interfaces
from std_msgs.msg import String
from interfaces_subzero.srv import SendString
from interfaces_subzero.srv import LoggerString
# Python packages
import datetime
# Submodules
from subzero_dec22.submodules.DeltaTime import DeltaTime

class PingPongRadio(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('PingPongRadio')
        self.client_Logger = self.create_client(LoggerString,"Logger_service")
        self.Client_Write2Logger("info","PingPongRadio: started")
        self.t0Ping = datetime.datetime.now()
        self.TelemetryInterval = 5
        self.IgnoreSerial = "listen"
        # ---Initiate Subscribers
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        # ---Initiate Services

        # ---Initiate Clients
        self.client_Radio = self.create_client(SendString,"Radio_service")
        self.client_ManualDrive = self.create_client(SendString,"ManualDrive_service")
        # ---Initiate Publishers

        # ---Initiate Timers
        # self.create_timer(3, self.timer_pingpong)
        self.create_timer(1, self.timer_checkTime)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def timer_checkTime(self):
        waiting = DeltaTime(self.t0Ping)
        if waiting > self.TelemetryInterval+2:
            self.Client_Write2ManualDrive("No Radio Feedback")
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    # ---publisher callback: publishes the Radio string message through topic_MsgFromRadio
    def radio_listener(self, msg):
        try:
            self.t0Ping = datetime.datetime.now()
            RadioMsg = msg.data
            comma_splt = RadioMsg.split(",")
            RadioCommand = comma_splt[0]
            RadioSubCommand = comma_splt[1:]
            if RadioCommand == "CMDSetTelemetryInterval":
                self.t0Ping = datetime.datetime.now()
                self.Client_Write2Arduino(RadioMsg)
                self.TelemetryInterval = float(RadioSubCommand[0])
        except Exception as e:
            self.Client_Write2Logger("error","PingPongRadio: radio_listener")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        try:
            request = SendString.Request()
            request.data = MsgForRadio
            self.client_Radio.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","PingPongRadio: Client_Write2Radio")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2ManualDrive(self,MsgForManualDrive):
        try:
            request = SendString.Request()
            request.data = MsgForManualDrive
            self.client_ManualDrive.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","PingPongRadio: Client_Write2ManualDrive")
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
    node = PingPongRadio() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
