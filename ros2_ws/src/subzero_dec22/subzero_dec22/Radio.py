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
from interfaces_subzero.srv import LoggerString
# Python packages
import serial
import datetime
import subprocess
import time

class Radio(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('Radio')
        self.client_Logger = self.create_client(LoggerString,"Logger_service")
        self.Client_Write2Logger("info","Radio: started")
        # ---Initiate the Radio device usb port
        while True:
            try:
                self.Client_Write2Logger("info","RADIO: crating object")
                self.serRadio = serial.Serial('/dev/ttyUSB0',38400,timeout=1)
                self.serRadio.reset_input_buffer()
                self.serRadio.reset_output_buffer()
                self.Client_Write2Logger("info","RADIO: object created")
                break
            except Exception as e:
                self.Client_Write2Logger("critical","RADIO: failed to create object")
                self.Client_Write2Logger("exception",str(e))
                time.sleep(2)
        # ---Initiate Services
        self.server_ = self.create_service(SendString,"Radio_service",self.WriteToRadio_Server)
        # ---Initiate Clients
        
        # ---Initiate Publishers
        self.RadioPublisher = self.create_publisher(String,'RadioPub_topic', 10)
        # ---Initiate Timers
        self.create_timer(0.2, self.timer_ReadFromRadio_Publisher)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    # ---publisher callback: publishes the Radio string message through topic_MsgFromRadio
    def timer_ReadFromRadio_Publisher(self):
        try:
            msg = String()
            if self.serRadio.in_waiting > 0:
                trash = self.serRadio.read_until(b'#')
                rawRadioMsg = self.serRadio.read_until(b'$')
                self.serRadio.flushInput()
                self.serRadio.reset_input_buffer()
                rawRadioMsg = rawRadioMsg.decode()
                rawRadioMsg = rawRadioMsg.replace('#','')
                rawRadioMsg = rawRadioMsg.replace('$','')
                if rawRadioMsg != 'pong':
                    self.Client_Write2Logger("info",f"RADIO: received {rawRadioMsg}")
                if "ARK_OS" in rawRadioMsg:
                    BashCommand = rawRadioMsg.replace('ARK_OS,','')
                    subprocess.Popen([BashCommand],shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)
                else:
                    if len(rawRadioMsg)>3:
                        msg.data = rawRadioMsg
                        self.RadioPublisher.publish(msg)
        except Exception as e:
            self.Client_Write2Logger("error","RADIO: unable to read the message")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    # ---server callback --> sends string to Radio if client asks 
    def WriteToRadio_Server(self, request, response):
        try:
            self.serRadio.reset_output_buffer()
            self.serRadio.write(bytes("#"+request.data+"$\r\n",'utf-8'))
            self.serRadio.flushOutput()
            if request.data != 'ping':
                self.Client_Write2Logger("info",f"RADIO: sent {'#'+request.data+'$'}")
            response.msg_sent = True
            return response
        except Exception as e:
            self.Client_Write2Logger("error","RADIO: WriteToRadio_Server")
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
    node = Radio() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
