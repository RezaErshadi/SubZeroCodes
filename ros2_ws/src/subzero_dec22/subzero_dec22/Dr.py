#!/usr/bin/env python3
"""

"""
import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module
# Interfaces
from std_msgs.msg import Bool
# Python packages
import time
import subprocess
import datetime

class Dr(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        # ---initialize the Node object
        super().__init__('Dr')
        time.sleep(5)
        now = datetime.datetime.now()
        self.t0 = [now,now,now,now,now]
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        # always listens to Radio through topic_MsgFromRadio
        self.create_subscription(Bool,'topic_HealthTrimble',self.trimble_listener,10)
        self.create_subscription(Bool,'topic_HealthRadio',self.radio_listener,10)
        self.create_subscription(Bool,'topic_HealthArduino',self.arduino_listener,10)
        self.create_subscription(Bool,'topic_HealthARK',self.ark_listener,10)
        self.create_subscription(Bool,'topic_HealthApRES',self.apres_listener,10)
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        # timer for health check
        self.create_timer(0.25,self.timer_ReRunNode)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def trimble_listener(self, msg):
        self.t0[0] = datetime.datetime.now()
    def radio_listener(self, msg):
        self.t0[1] = datetime.datetime.now()
    def arduino_listener(self, msg):
        self.t0[2] = datetime.datetime.now()
    def ark_listener(self, msg):
        self.t0[3] = datetime.datetime.now()
    def apres_listener(self, msg):
        self.t0[4] = datetime.datetime.now()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def timer_ReRunNode(self):
        now = datetime.datetime.now()
        nodes = ["Trimble","Radio","Arduino","ARK","ApRES"]
        for i in range(len(nodes)):
            dT = round( self.DeltaTime(self.t0[i],now) ,1)
            print(f"{nodes[i]}={dT}")
            if dT > 3:
                self.get_logger().info(f"{datetime.datetime.now()}--> {nodes[i]} is dead, Restarting the node")
                cmd = f"ros2 run subzero_v1 {nodes[i]}"
                self.get_logger().info(f"{datetime.datetime.now()}--> {cmd}")
                subprocess.Popen([cmd],shell=True,stdin=None,stdout=None,stderr=None,close_fds=True)
                self.t0[i] = datetime.datetime.now()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """           
    def DeltaTime(self,t0,now):
        d = now - t0
        return d.seconds + (d.microseconds)/1000000
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
def main(args=None):
    rclpy.init(args=args) # initialize ROS communications
    node = Dr() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
