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
from interfaces_subzero.srv import SendString
from interfaces_subzero.srv import SpeedValues
from interfaces_subzero.srv import LoggerString
# Python packages
import datetime
# Submodules
from subzero_dec22.submodules.constrain import constrain
from subzero_dec22.submodules.DeltaTime import DeltaTime

class WriteSpeed(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('WriteSpeed')
        self.client_Logger = self.create_client(LoggerString,"Logger_service")
        self.Client_Write2Logger("info","WriteSpeed: started")
        # ---Initiate Parameters
        self.desiredV = 0.0
        self.desiredW = 0.0
        self.currentV = 0.0
        self.currentW = 0.0

        self.LinearIncrement = 10
        self.DifferentialIncrement = 28.0
        self.ActualSpeedUpdateTime = datetime.datetime.now()
        self.ActualSpeedUpdateInterval = 0.05 # do not change speed more often than every 50 ms
        
        self.MAXV = 100.0
        self.MAXW = 140.0 # for 12 bit resolution for analogWrite

        self.Woffset = 0.0
        self.Wfactor = 1.0
        self.Ffactor = 1717
        self.upperlimit = 3777
        self.lowerlimit = 335
        self.cent = 2056
        # ---Initiate Subscribers

        # ---Initiate Services
        self.server_ = self.create_service(SpeedValues,"WriteSpeed_service",self.WriteSpeed_Server)
        # ---Initiate Clients
        self.client_Radio = self.create_client(SendString,"Radio_service")
        self.client_Arduino = self.create_client(SendString,"Arduino_service")
        self.client_VW_Telemetry = self.create_client(SpeedValues,"VW_Telemetry_service")
        # ---Initiate Timers

    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def WriteSpeed_Server(self,request, response):
        try:
            self.desiredV = request.v_linear
            self.desiredW = request.w_angular
            # self.Client_Write2Logger("info",f"WriteSpeed: new desired speed v:{self.desiredV}, w:{self.desiredW}")
            self.ActualSpeedUpdate()
            response.msg_sent = True
            return response
        except Exception as e:
            self.Client_Write2Logger("error","WriteSpeed: WriteSpeed_Server")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ActualSpeedUpdate(self):
        try:
            if DeltaTime(self.ActualSpeedUpdateTime) > self.ActualSpeedUpdateInterval:
                self.ActualSpeedUpdateTime = datetime.datetime.now()
                # constrain W if V too low
                if abs(self.currentV) < 10:
                    self.desiredW = 0.0
                # define and constrain the new V and W values    
                self.currentV = constrain(self.desiredV, -self.MAXV, self.MAXV)	
                self.currentW = constrain(self.desiredW, -self.MAXW, self.MAXW)
                self.Client_VW2Telemetry(self.currentV,self.currentW)
                # self.Client_Write2Logger("info",f"WriteSpeed: new current speed v:{self.currentV}, w:{self.currentW}")
                self.WriteSpeedOnArduino()
        except Exception as e:
            self.Client_Write2Logger("error","WriteSpeed: ActualSpeedUpdate")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def WriteSpeedOnArduino(self):
        try:
            if (self.currentV>=0):
                vpFL = ((((self.currentV*80)/100)*self.Ffactor)/100)+self.cent
            else:
                vpFL = ((((self.currentV*83)/100)*self.Ffactor)/100)+self.cent
            vpBL = ((self.currentV*self.Ffactor)/100)+self.cent
            vpFR = ((self.currentV*self.Ffactor)/100)+self.cent
            vpBR = ((self.currentV*self.Ffactor)/100)+self.cent
            wPrime = self.Wfactor * (self.currentW + self.Woffset)
            vFL = constrain(vpFL + wPrime,self.lowerlimit,self.upperlimit)
            vBL = constrain(vpBL + wPrime,self.lowerlimit,self.upperlimit)
            vFR = constrain(vpFR - wPrime,self.lowerlimit,self.upperlimit)
            vBR = constrain(vpBR - wPrime,self.lowerlimit,self.upperlimit)
            # write actual values to output registers --> to motor controllers
            # self.get_logger().info(f"calculated current: {vFL},{vBL},{vFR},{vBR}")
            WriteVelocity = f"CMDSetSpeed,{vFL},{vBL},{vFR},{vBR}"
            self.Client_Write2Arduino(WriteVelocity)
            # self.Client_Write2Logger("info",f"WriteSpeed: individual wheels FL:{vFL}, BL:{vBL}, FR{vFR}, BR{vBR}")
        except Exception as e:
            self.Client_Write2Logger("error","WriteSpeed: WriteSpeedOnArduino")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        try:
            request = SendString.Request()
            request.data = MsgForRadio
            self.client_Radio.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","WriteSpeed: Client_Write2Radio")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Arduino(self,MsgForArduino):
        try:
            request = SendString.Request()
            request.data = MsgForArduino
            self.client_Arduino.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","WriteSpeed: Client_Write2Arduino")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_VW2Telemetry(self,V,W):
        try:
            request = SpeedValues.Request()
            request.v_linear = V
            request.w_angular = W
            self.client_VW_Telemetry.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","WriteSpeed: Client_VW2Telemetry")
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
    node = WriteSpeed() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
