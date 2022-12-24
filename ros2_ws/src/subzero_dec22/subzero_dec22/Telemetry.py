#!/usr/bin/env python3
"""
This node always listens to the messages from Arduino and Radio
and make important decisions.
"""

import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module
# Interfaces
from std_msgs.msg import String
from interfaces_subzero.msg import MyGPSmsg
from interfaces_subzero.srv import SendString
from interfaces_subzero.srv import SpeedValues
from interfaces_subzero.srv import NextTarget
from interfaces_subzero.srv import LoggerString
# Python packages
import datetime
# Submodules
from subzero_dec22.submodules.DeltaTime import DeltaTime

class Telemetry(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        # ---Initiate the Node object
        super().__init__('Telemetry')
        self.client_Logger = self.create_client(LoggerString,"Logger_service")
        self.Client_Write2Logger("info","Telemetry: started")
        # --- Initiate Parameters
        self.RadioPingTimer = datetime.datetime.now()
        self.t0GPSage = datetime.datetime.now()
        self.t0Telemetry = datetime.datetime.now()
        self.TelemetryInterval = 5
        self.IgnoreSerial = "listen"
        self.BatteryVoltage = 0.0
        self.gpsTime = 0.0
        self.gpsAge = 0.0
        self.RoverLat = 0.0
        self.RoverLon = 0.0
        self.RoverAlt = 0.0
        self.RoverSpeed = 0.0
        self.RoverCourse = 0.0
        self.currentV = 0.0
        self.currentW = 0.0
        self.iNextTarget = 0.0
        self.NextTargetLat = 0.0
        self.NextTargetLon = 0.0
        self.BearingToNextTarget = 0.0
        self.DistToNextTarget = 0.0
        self.DistFromLastRef = 0.0
        self.GPSQualityIndex = 0
        # ---Initiate Subscriber
        self.ListenToGPS = self.create_subscription(MyGPSmsg,'GPSPub_topic',self.UpdateGPS,10)
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        self.ListenToArduino = self.create_subscription(String,'ArduinoPub_topic',self.arduino_listener,10)
        # ---Initiate Services
        self.server_VW = self.create_service(SpeedValues,"VW_Telemetry_service",self.GetVW_info_Server)
        self.server_Target = self.create_service(NextTarget,"Target_Telemetry_service",self.GetTarget_info_Server)
        # ---Initiate Clients
        self.client_Radio = self.create_client(SendString,"Radio_service")
        self.client_Arduino = self.create_client(SendString,"Arduino_service")
        # ---Initiate Publishers

        # ---Initiate Timers
        self.create_timer(0.01,self.TransmitTelemetry)
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def TransmitTelemetry(self):
        try:
            if (self.IgnoreSerial == "listen") and (DeltaTime(self.t0Telemetry) >= self.TelemetryInterval):
                self.t0Telemetry = datetime.datetime.now()
                t1 = f"{self.gpsAge},"
                t2 = f"{self.gpsTime},"
                t3 = f"{self.RoverLat},"
                t4 = f"{self.RoverLon},"
                t5 = f"{self.RoverAlt},"
                t6 = f"{self.RoverSpeed},"
                t7 = f"{self.RoverCourse},"
                t8 = f"{self.BatteryVoltage},"
                t9 = f"{self.currentV},"
                t10 = f"{self.currentW},"
                t11 = f"{self.iNextTarget+1},"
                t12 = f"{self.NextTargetLat},"
                t13 = f"{self.NextTargetLon},"
                t14 = f"{self.BearingToNextTarget},"
                t15 = f"{self.DistToNextTarget},"
                t16 = f"{self.DistFromLastRef},"
                t17 = f"{self.GPSQualityIndex}"
                TelMsg = t1+t2+t3+t4+t5+t6+t7+t8+t9+t10+t11+t12+t13+t14+t15+t16+t17
                self.Client_Write2Radio("TELEMETRY,"+TelMsg)
        except Exception as e:
            self.Client_Write2Logger("error","TELEMETRY: TransmitTelemetry")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def radio_listener(self, msg):
        try:
            RadioMsg = msg.data
            comma_splt = RadioMsg.split(",")
            RadioCommand = comma_splt[0]
            RadioSubCommand = comma_splt[1:]
            if RadioCommand == "CMDSetIgnoreInstrument":
                self.Client_Write2Arduino(RadioMsg)
                self.IgnoreSerial = RadioSubCommand[0] #ignore/listen
                self.Client_Write2Logger("info",f"TELEMETRY: change status to {self.TelemetryInterval}")
            elif RadioCommand == "CMDSetTelemetryInterval":
                self.Client_Write2Arduino(RadioMsg)
                self.TelemetryInterval = float(RadioSubCommand[0])
                self.Client_Write2Logger("info",f"TELEMETRY: change interval to {self.TelemetryInterval} seconds")
        except Exception as e:
            self.Client_Write2Logger("error","TELEMETRY: radio_listener")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def arduino_listener(self, msg):
        try:
            ArduinoMsg = msg.data
            self.ArduinoList = ArduinoMsg.split(",")
            self.ArduinoCommand = self.ArduinoList[0]
            self.ArduinoSubCommand = self.ArduinoList[1:]
            if self.ArduinoCommand == "TELArduino":
                self.BatteryVoltage = float(self.ArduinoSubCommand[0])
                # vFL = float(self.ArduinoSubCommand[1])
                # vBL = float(self.ArduinoSubCommand[2])
                # vFR = float(self.ArduinoSubCommand[3])
                # vBR = float(self.ArduinoSubCommand[4])
                # self.get_logger().info(f"Arduino: {vFL},{vBL},{vFR},{vBR}")
                # self.Client_Write2Logger("info",f"TELEMETRY: Arduino Listener: battery voltage {self.BatteryVoltage}")
                # self.Client_Write2Logger("info",f"TELEMETRY: Arduino Listener: speed {vFL},{vBL},{vFR},{vBR}")
        except Exception as e:
            self.Client_Write2Logger("error","TELEMETRY: arduino_listener")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """            
    def UpdateGPS(self,msg):  
        try: 
            self.gpsAge  = DeltaTime(self.t0GPSage)
            self.t0GPSage = datetime.datetime.now()
            self.gpsTime = msg.time 
            self.RoverLat = msg.lat
            self.RoverLon = msg.lon
            if msg.iqual != 9:
                self.GPSQualityIndex = msg.iqual
                self.RoverAlt = msg.alt
            if msg.iqual == 9:
                self.RoverCourse = msg.course
                self.RoverSpeed = msg.speed
        except Exception as e:
            self.Client_Write2Logger("error","TELEMETRY: UpdateGPS")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """     
    def GetVW_info_Server(self,request, response):
        try:
            self.currentV = request.v_linear
            self.currentW = request.w_angular
            response.msg_sent = True
            return response
        except Exception as e:
            self.Client_Write2Logger("error","TELEMETRY: GetVW_info_Server")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """     
    def GetTarget_info_Server(self,request, response):
        try:
            self.iNextTarget = request.i_next_target
            self.NextTargetLat = request.next_target_lat
            self.NextTargetLon = request.next_target_lon
            self.BearingToNextTarget = request.bearing_to_next_target
            self.DistToNextTarget = request.dist_to_next_target
            self.DistFromLastRef = request.dist_from_last_ref
            response.msg_sent = True
            return response
        except Exception as e:
            self.Client_Write2Logger("error","TELEMETRY: GetTarget_info_Server")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        try:
            request = SendString.Request()
            request.data = MsgForRadio
            self.client_Radio.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","TELEMETRY: Client_Write2Radio")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Arduino(self,MsgForArduino):
        try:
            request = SendString.Request()
            request.data = MsgForArduino
            self.client_Arduino.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","TELEMETRY: Client_Write2Arduino")
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
    node = Telemetry() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
