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
from interfaces_subzero.msg import MyGPSmsg
from interfaces_subzero.srv import SpeedValues
from interfaces_subzero.srv import NextTarget
# Python packages
import time
import datetime
import pyproj
import math
# Submodules
from subzero_dec22.submodules import constrain
from subzero_dec22.submodules import DeltaTime

"""
float 		robotLength 	= 1.05;
float	  	robotWidth		= 1.20; //width between center of wheels on same axle
"""

class AutoPilot(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('AutoPilot')
        # ---Initiate Parameters
        self.geodesic = pyproj.Geod(ellps='WGS84')

        self.SurveyMode = False
        self.DriveMode = "Manual"
        self.SubDrive = "Stop"
        self.LastRef = [0,0]
        self.DistFromRef = 0.0

        self.BatteryVoltage = 0
        self.enableLowBatteryMode = True
        self.lowBatteryMode = False

        self.desiredV = 0.0
        self.desiredW = 0.0
        self.currentV = 0.0
        self.currentW = 0.0
        
        self.MAXV = 100.0
        self.MAXW = 140.0 # for 12 bit resolution for analogWrite
        self.MAXWI = 20.0
        self.AutonomousMaxSpeed = 45.0
        self.SAR_Max_Speed = 25

        self.EarthRadiuskm = 6378.14
        self.waypointRadius = 25
        self.navigationRadius = 15

        self.P_Coeff = 6.0
        self.I_Coeff = 0.0
        self.SPEED_P_COEFF = 0.5 # this is a very first order approximation - please adjust depending on value of AutonomousMaxSpeed
        self.ROVER_POSITION_ERROR = 0.1 # again, adjust depending on desired position? 

        self.DesiredUpdateTime = datetime.datetime.now()
        self.DesiredUpdateInterval = 0.1
        self.DestinationUpdateTime = datetime.datetime.now()
        self.DestinationUpdateInterval = 0.2
        # ---Initiate Subscribers
        self.ListenToGPS = self.create_subscription(MyGPSmsg,'GPSPub_topic',self.UpdateGPS,10)
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        self.ListenToArduino = self.create_subscription(String,'ArduinoPub_topic',self.arduino_listener,10)
        # ---Initiate Services
        self.server_ = self.create_service(SendString,"Driving_service",self.Driving_Server)
        # ---Initiate Clients
        self.client_Radio = self.create_client(SendString,"Radio_service")
        self.client_ApRES = self.create_client(SendString,"ApRES_service")
        self.client_WriteSpeed = self.create_client(SpeedValues,"WriteSpeed_service")
        self.client_Target_Telemetry = self.create_client(NextTarget,"Target_Telemetry_service")
        # ---Initiate Timers
        self.create_timer(0.2,self.timer_AutoDrive)
        # Initiate GPS and Telemetry timer
        self.t0GPSage = datetime.datetime.now()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def UpdateGPS(self, msg):
        d = datetime.datetime.now() - self.t0GPSage
        self.gpsAge = round(d.seconds + (d.microseconds)/1000000,1)
        self.t0GPSage = datetime.datetime.now()
        self.gpsTime = msg.time 
        self.LastPose = [msg.lat,msg.lon]
        self.LastCourse = msg.course
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def timer_AutoDrive(self):
        if (self.DriveMode == "Autonomous") and (self.nTargetPoints > 0):
            if self.SubDrive == "Go":
                self.Destination_Speed_Update()
                self.Client_Target2Telemetry()
            elif self.SubDrive == "Action":
                self.ActionUpdate()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ActionUpdate(self):
        self.DoubleCheckStop()
        act = self.NextTargetAct[self.iAction]
        self.iAction += 1
        if act == "Shoot": # ApRES Burst
            self.Client_Write2ApRES("Burst")
            self.SubDrive = "Wait"
        elif act == "Ref": # Update reference point
            self.LastRef = self.LastPose
            self.SurveyMode = "SAR"
            self.SubDrive = "Action"
        elif act == "Next":
            if self.SurveyMode == "SAR":
                self.iNextTarget = -1
            else:
                self.iNextTarget += 1
            self.SubDrive = "Go"
        elif act == "Done":
            self.Client_Write2Radio("Survey: Done")
            self.EmStop()
            self.SubDrive = "Stop"
        if self.iAction == len(self.NextTargetAct):
            self.iAction = 0
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Destination_Speed_Update(self):
        if DeltaTime(self.DestinationUpdateTime) > self.DestinationUpdateInterval:
            self.DestinationUpdateTime = datetime.datetime.now()
            # update rover position and course
            self.RoverNow = self.LastPose
            self.RoverCourseNow = self.LastCourse
            self.NextTargetLat = self.TargetListLat[self.iNextTarget]
            self.NextTargetLon = self.TargetListLon[self.iNextTarget]
            self.NextTargetAct = self.TargetListAct[self.iNextTarget]
            target = [self.NextTargetLat,self.NextTargetLon]
            # find distance and foraward and backward bearing to te next target from current position
            self.UpdateDist_Az(target)
            if (self.SurveyMode == "SAR") and (self.DistFromRef >= self.sarSpacing-(self.sarSpacing*0.1)):
                self.Client_Write2Radio(f"Rover reached to point a new reference point - {self.DistFromRef}")
                self.ActionStop()
            else:
                if self.DistToNextTarget < self.waypointRadius:
                    self.Client_Write2Radio(f"Rover reached to point #{self.iNextTarget+1}/{self.nTargetPoints}")
                    # What should the rover do after it reached to the destination
                    if self.NextTargetAct[0] == "Stop":
                        self.ActionStop()
                    elif self.NextTargetAct[0] == "Next":
                        self.SubDrive = "Go"
                        self.iNextTarget += 1
                        self.DesiredSpeedUpdate()
                else:
                    self.DesiredSpeedUpdate()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def UpdateDist_Az(self,target):
        self.BearingToNextTarget,self.back_azimuth,self.DistToNextTarget = self.geodesic.inv(self.RoverNow[1],self.RoverNow[0],target[1],target[0])
        _,_,self.DistFromLastRef = self.geodesic.inv(self.RoverNow[1],self.RoverNow[0],self.LastRef[1],self.LastRef[0])
        self.BearingToNextTarget = round(self.BearingToNextTarget,2)
        self.back_azimuth = round(self.back_azimuth,2)
        self.DistToNextTarget = round(self.DistToNextTarget,2)
        self.DistFromRef = round(self.DistFromRef,2)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def EmStop(self):
            self.desiredW = 0
            self.desiredV = 0
            self.Client_WriteSpeed(self.desiredV,self.desiredW)
            self.DriveMode = "Manual"
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def DoubleCheckStop(self):
            self.desiredW = 0
            self.desiredV = 0
            self.Client_WriteSpeed(self.desiredV,self.desiredW)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ActionStop(self):
            self.desiredW = 0
            self.desiredV = 0
            self.Client_WriteSpeed(self.desiredV,self.desiredW)
            if self.SurveyMode == "SAR":
                self.NextTargetAct = ["Stop","Shoot","Ref","Next"]
                self.iAction = 1
            else:
                self.iAction += 1
            self.SubDrive = "Action"
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def GetHeadingError(self,initial, final):
        diff = final - initial
        absDiff = abs(diff)
        # would like to rewrite this for intelligibility
        if absDiff <= 180.0:
            if absDiff == 180.0:
                return absDiff
            else:
                return diff
        elif final > initial:
            return (absDiff - 360.0)
        else:
            return (360.0 - absDiff)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def DesiredSpeedUpdate(self):
        IntegralHeadingError = 0.0
        headingError = 0.0
        if DeltaTime(self.DesiredUpdateTime) > self.DesiredUpdateInterval:
            self.DesiredUpdateTime = datetime.datetime.now()
            if self.gpsAge > 5:
                self.EmStop()
                self.Client_Write2Radio("GPS failed")
            elif (self.enableLowBatteryMode == True) and (self.lowBatteryMode):
                self.EmStop()
                self.Client_Write2Radio("Rover Stopped: low battery")
            else:
                # convert to decimal radians
                latitude1  = math.pi * self.NextTargetLat / 180.0
                longitude1 = math.pi * self.NextTargetLon / 180.0
                if self.DistToNextTarget > self.navigationRadius:
                    # implements the "sliding waypoint" or "pure pursuit" algorithm for path following
                    goalDist = self.DistToNextTarget - self.navigationRadius
                    d = goalDist / 1000.0
                    brng = math.pi * self.back_azimuth / 180.0
                    # dead reckon location "goaldist" distance from goal along path to previous goal
                    latitude2 = math.asin(math.sin(latitude1) * math.cos(d / self.EarthRadiuskm) + math.cos(latitude1) * math.sin(d / self.EarthRadiuskm) * math.cos(brng))
                    longitude2 = longitude1 + math.atan2(math.sin(brng) * math.sin(d / self.EarthRadiuskm) * math.cos(latitude1), math.cos(d / self.EarthRadiuskm) - math.sin(latitude1) * math.sin(latitude2))
                    # convert back to degrees
                    TargetLat = latitude2 * 180.0 / math.pi
                    TargetLON = longitude2 * 180.0 / math.pi
                else:
                    TargetLat = latitude1
                    TargetLON = longitude1
                self.UpdateDist_Az([TargetLat,TargetLON])
                # calculate the error between actual heading and bearing to goal
                headingError = self.GetHeadingError(self.RoverCourseNow, self.BearingToNextTarget)
                # calculate integral term
                IntegralHeadingError = IntegralHeadingError + ((self.DesiredUpdateInterval / 1000.0) * headingError)
                # prevents dividing by zero for I_Coeff = 0
                if self.I_Coeff == 0:
                    IntegralHeadingError = 0.0
                else:
                    # constrain to prevent windup
                    IntegralHeadingError = constrain(IntegralHeadingError, -self.MAXWI/self.I_Coeff,self.MAXWI/self.I_Coeff)
                # set desired V and W
                self.desiredW = self.P_Coeff * headingError + self.I_Coeff * IntegralHeadingError
                self.desiredW = constrain(self.desiredW, -self.MAXW, self.MAXW) # ensure that 'abs(desiredW) < MAXW'
                V_Max = self.AutonomousMaxSpeed
                if self.SurveyMode == "SAR":
                    V_Max = self.SAR_Max_Speed
                self.desiredV = self.SPEED_P_COEFF * self.DistToNextTarget * V_Max
                self.desiredV = self.constrain(self.desiredV, 0, V_Max)
                if self.DistToNextTarget <= self.ROVER_POSITION_ERROR:
                    self.desiredV = 0
                self.Client_WriteSpeed(self.desiredV,self.desiredW)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def radio_listener(self, msg):
        RadioMsg = msg.data
        RadioList = RadioMsg.split(",")
        RadioCommand = RadioList[0]
        RadioSubCommand = RadioList[1:]
        #------------------------------
        if RadioCommand == "CMDDriveMode":
            self.DriveMode = RadioSubCommand[0]
            if RadioSubCommand[0]== "Autonomous":
                self.EmStop()
                self.LastRef = self.RoverNow
                self.iNextTarget = 0
                self.desiredV = self.AutonomousMaxSpeed #start driving forward for heading
                self.ActionRequiredFlag = False
                self.DriveMode = RadioSubCommand[0]
                self.SubDrive = "Go"
        #------------------------------
        elif RadioCommand == "CMDEmStop":
            self.EmStop()
        #------------------------------
        elif RadioCommand == "UpdateHome":
            # Update Home location
            self.ThisIsHome = self.RoverNow
            MsgForRadio = f"HomeUpdated,{self.ThisIsHome[0]},{self.ThisIsHome[1]}"
            self.Client_Write2Radio(MsgForRadio)
        #------------------------------
        elif self.RadioCommand == "ComeHome":
            self.nTargetPoints = 1
            self.TargetListLat = [float(self.ThisIsHome[0])]
            self.TargetListLon = [float(self.ThisIsHome[1])]
            self.TargetListAct = [["Stop","Done"]]
            self.SurveyMode = "ComeHome"
        #------------------------------
        elif "Survey" in RadioCommand:
            self.ListTargetPoints()
            if RadioCommand == "AdventureSurvey":
                self.TargetListAct = [["Next"]] * self.nTargetPoints
                self.TargetListAct[-1] = ["Stop","Done"]
                self.SurveyMode = "Adventure"
            #------------------------------
            elif RadioCommand == "PolarimetricSurvey":
                self.TargetListAct = [["Stop","Shoot","Next"]] * self.nTargetPoints
                self.TargetListAct[-1] = ["Stop","Shoot","Done"]
                self.SurveyMode = "Polarimetric"
            #------------------------------
            elif RadioCommand == "SARSurvey":
                self.TargetListAct = [["Next"],["Stop","Shoot","Ref","SarNext"],["Stop","Shoot","Done"]]
                self.SurveyMode = "SAR_0"
            #------------------------------
        elif RadioCommand == "CMDSetLowBatt":
            if RadioSubCommand[0] == "true":
                self.enableLowBatteryMode = True
            else:
                self.enableLowBatteryMode = False
        #------------------------------
        elif RadioCommand == "CMDAutonomousMaxSpeed":
            self.AutonomousMaxSpeed = float(RadioSubCommand[0])
        #------------------------------
        elif RadioCommand == "CMDSARMaxSpeed":
            self.AutonomousMaxSpeed = float(RadioSubCommand[0])
        #------------------------------
        elif RadioCommand == "CMDSetNavRad":
            self.navigationRadius = float(RadioSubCommand[0])
        #------------------------------
        elif RadioCommand == "CMDSetWayPointRad":
            self.waypointRadius = float(RadioSubCommand[0])
        #------------------------------
        elif RadioCommand == "CMDSetPID":
            self.P_Coeff = float(RadioSubCommand[0])
            self.I_Coeff = float(RadioSubCommand[1])
            self.SPEED_P_COEFF = float(RadioSubCommand[2])
            self.ROVER_POSITION_ERROR = float(RadioSubCommand[3])
        #------------------------------
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def arduino_listener(self, msg):
        ArduinoMsg = msg.data
        self.ArduinoList = ArduinoMsg.split(",")
        self.ArduinoCommand = self.ArduinoList[0]
        self.ArduinoSubCommand = self.ArduinoList[1:]
        if self.ArduinoCommand == "TELArduino":
            self.BatteryVoltage = float(self.ArduinoSubCommand[0])
        # Switch to lowBatteryMode if the voltage drops below 49
        if (self.lowBatteryMode==False and self.BatteryVoltage < 49.0):
            self.lowBatteryMode = True
            self.LowBatteryTime = datetime.datetime.now()
        # Turn off the lowBatteryMode if the voltage is above 51
        elif (self.lowBatteryMode and self.batteryVoltage > 51.0) and (DeltaTime(self.LowBatteryTime) > self.LowBatteryInterval):
            self.lowBatteryMode = False
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ListTargetPoints(self):
        self.TargetListLat = []
        self.TargetListLon = []
        self.TargetListAct = []
        if self.RadioCommand == "SARSurvey":
            self.sarSpacing = float(self.RadioSubCommand[0])
            self.nTargetPoints = 3
        else:
            self.nTargetPoints = int(self.RadioSubCommand[0])
        LatLonAct = self.RadioSubCommand[1:]
        for i in range(self.nTargetPoints):
            ii = 2*i
            self.TargetListLat.append(LatLonAct[ii])
            self.TargetListLon.append(LatLonAct[ii+1])
        self.TargetListLat = list(map(float,self.TargetListLat))
        self.TargetListLon = list(map(float,self.TargetListLon))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Driving_Server(self,request, response):
        msg = request.data
        if msg == "ApRES: Done":
            self.SubDrive = "Action"
        response.msg_sent = True
        return response
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Target2Telemetry(self):
        request = SpeedValues.Request()
        request.i_next_target = self.iNextTarget
        request.next_target_lat = self.NextTargetLat
        request.next_target_lon = self.NextTargetLon
        request.bearing_to_next_target = self.BearingToNextTarget
        request.dist_to_next_target = self.DistToNextTarget 
        request.dist_from_last_ref = self.DistFromLastRef
        self.client_Target_Telemetry.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_WriteSpeed(self,desiredV,desiredW):
        request = SpeedValues.Request()
        request.v_linear = desiredV
        request.w_angular = desiredW
        self.client_WriteSpeed.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        request = SendString.Request()
        request.data = MsgForRadio
        self.client_Radio.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2ApRES(self,MsgForApRES):
        request = SendString.Request()
        request.data = MsgForApRES[0]
        self.client_ApRES.call_async(request)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
def main(args=None):
    rclpy.init(args=args) # initialize ROS communications
    node = AutoPilot() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
