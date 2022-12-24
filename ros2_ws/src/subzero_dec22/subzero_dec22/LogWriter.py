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
from interfaces_subzero.srv import LoggerString
# Python packages
import datetime
import logging
import os
# Submodules

class LogWriter(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('LogWriter')
        # ---Initiate Parameters
        now = datetime.datetime.now()
        SubZEroLogFolder = '/home/subzero/ros2lf/SubZero/drive/'
        if not os.path.exists(SubZEroLogFolder):
            os.makedirs(SubZEroLogFolder)
        nf = len(os.listdir(SubZEroLogFolder))
        LogFileName = f"{nf+1}_SubZeroDrive_{now.strftime('%m%d%Y_%H%M%S')}.txt"
        LogFilePath = SubZEroLogFolder + LogFileName
        print(LogFilePath)
        logging.basicConfig(filename=LogFilePath, filemode='w', format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self.logger=logging.getLogger()
        self.logger.setLevel(logging.DEBUG)
        self.logger.info("Logger Initiated")
        # ---Initiate Subscribers

        # ---Initiate Services
        self.server_logger = self.create_service(LoggerString,"Logger_service",self.LoggerServerCallback)
        # ---Initiate Clients

        # ---Initiate Timers

    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def LoggerServerCallback(self,request, response):
        logtype = request.type
        logmsg = request.data
        if logtype == "debug":
            self.logger.debug(logmsg)
        elif logtype == "info":
            self.logger.info(logmsg)
        elif logtype == "warning":
            self.logger.warning(logmsg)
        elif logtype == "error":
            self.logger.error(logmsg)
        elif logtype == "exception":
            self.logger.error("Exception occurred "+logmsg, exc_info=True)
        elif logtype == "critical":
            self.logger.critical(logmsg)
        response.msg_sent = True
        return response
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
def main(args=None):
    rclpy.init(args=args) # initialize ROS communications
    node = LogWriter() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
