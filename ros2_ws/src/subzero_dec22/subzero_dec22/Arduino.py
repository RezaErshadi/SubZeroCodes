#!/usr/bin/env python3
"""
This node opens a serial port for Arduino
It always listen to the messages coming from the arduino and publishes them via:
topic_MsgFromArduino
It also listens to any messages from other nodes which must be sent into the arduino via:
topic_MsgToArduino
"""
import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module
# Interfaces
from std_msgs.msg import String
from interfaces_subzero.srv import SendString
from interfaces_subzero.srv import LoggerString
# Python packages
import serial
import datetime
import time

class Arduino(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---initialize the Node object
        super().__init__('Arduino')
        self.client_Logger = self.create_client(LoggerString,"Logger_service")
        self.Client_Write2Logger("info","Arduino: started")
        # ---Initiate the Arduino device usb port
        while True:
            try:
                self.Client_Write2Logger("info","ARDUINO: crating object")
                self.serArduino = serial.Serial('/dev/ttyACM0',38400,timeout=1)
                self.serArduino.reset_input_buffer()
                self.serArduino.reset_output_buffer()
                self.Client_Write2Logger("info","ARDUINO: object created")
                break
            except Exception as e:
                self.Client_Write2Logger("error","ARDUINO: failed to create object")
                self.Client_Write2Logger("exception",str(e))
                time.sleep(2)
        # ---Initiate Services
        self.server_ = self.create_service(SendString,"Arduino_service",self.WriteToArduino_Server)
        # ---Initiate Clients

        # ---Initiate Publishers
        self.ArduinoPublisher = self.create_publisher(String,'ArduinoPub_topic', 10)
        # ---Initiate Timers
        self.create_timer(0.2, self.timer_ReadFromArduino_Publisher)
        # self.create_timer(4.13, self.timer_CheckRover)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """    
    # ---publisher callback: publishes the Arduino string message through topic_MsgFromArduino
    def timer_ReadFromArduino_Publisher(self):
        try:
            msg = String()
            if self.serArduino.in_waiting > 0:
                trash = self.serArduino.read_until(b'#')
                rawArduinoMsg = self.serArduino.read_until(b'$')
                self.serArduino.flushInput()
                self.serArduino.reset_input_buffer()
                rawArduinoMsg = rawArduinoMsg.decode()
                rawArduinoMsg = rawArduinoMsg.replace('#','')
                rawArduinoMsg = rawArduinoMsg.replace('$','')
                self.Client_Write2Logger("info",f"ARDUINO: received {rawArduinoMsg}")
                if len(rawArduinoMsg)>3:
                    msg.data = rawArduinoMsg
                    self.ArduinoPublisher.publish(msg)
        except Exception as e:
            self.Client_Write2Logger("error","ARDUINO: unable to read the message")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """    
    # ---publisher callback: publishes the Arduino string message through topic_MsgFromArduino
    # def timer_CheckRover(self):
    #     self.serArduino.reset_output_buffer()
    #     self.serArduino.write(bytes("#CMDChecking$$", 'utf-8'))
    #     self.serArduino.flushOutput()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    # ---server callback --> sends string to Arduino if client asks 
    def WriteToArduino_Server(self, request, response):
        try:
            self.serArduino.reset_output_buffer()
            self.serArduino.write(bytes("#"+request.data+"$$", 'utf-8'))
            self.serArduino.flushOutput()
            # self.Client_Write2Logger("info",f"ARDUINO: sent {'#'+request.data+'$$'}")
            response.msg_sent = True
            return response
        except Exception as e:
            self.Client_Write2Logger("error","ARDUINO: WriteToArduino_Server")
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
    node = Arduino() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
