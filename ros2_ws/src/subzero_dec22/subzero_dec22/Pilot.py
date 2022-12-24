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
from interfaces_subzero.srv import ApRESConf
# Python packages
import time
# Submodules

""" -------------------------------------------------- """
class Pilot(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('Pilot')
        self.DriveMode  == "Manual"
        # ---Initiate Parameters

        # ---Initiate Subscribers
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        # ---Initiate Services

        # ---Initiate Clients
        self.client_Radio = self.create_client(SendString,"Radio_service")
        self.client_WriteSpeed = self.create_client(SpeedValues,"WriteSpeed_service")
        # ---Initiate Timers
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def radio_listener(self, msg):
        RadioMsg = msg.data
        RadioList = RadioMsg.split(",")
        RadioCommand = RadioList[0]
        RadioSubCommand = RadioList[1:]
        if RadioCommand == "CMDDriveMode":
            self.DriveMode = RadioSubCommand[0]
        elif RadioCommand == "CMDEmStop":
            self.DriveMode  == "Manual"
        elif self.DriveMode  == "Manual":
            if RadioCommand == "CMDSetSpeed":
                V = float(RadioSubCommand[0])
                W = float(RadioSubCommand[1])
                self.Client_WriteSpeed(V,W)
            elif RadioCommand == "CMDStop":
                self.Stop()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Stop(self):
        self.Client_WriteSpeed(0,0)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        request = SendString.Request()
        request.data = MsgForRadio
        self.client_Radio.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_WriteSpeed(self,V,W):
        request = SpeedValues.Request()
        request.v_linear = V
        request.w_angular = W
        self.client_ApRES.call_async(request)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
def main(args=None):
    rclpy.init(args=args) # initialize ROS communications
    node = Pilot() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
