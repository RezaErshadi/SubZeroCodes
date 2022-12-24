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
from interfaces_subzero.srv import LoggerString
# Python packages
import time
import datetime
import pyproj
import math
# Submodules
from subzero_dec22.submodules.constrain import constrain
from subzero_dec22.submodules.DeltaTime import DeltaTime

"""
float 		robotLength 	= 1.05;
float	  	robotWidth		= 1.20; //width between center of wheels on same axle
"""

class AutoDrive(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('AutoDrive')
        self.client_Logger = self.create_client(LoggerString,"Logger_service")
        self.Client_Write2Logger("info","AutoDrive: started")
        # ---Initiate Parameters
        self.geodesic = pyproj.Geod(ellps='WGS84')

        self.SurveyMode = False
        self.DriveMode = "Manual"
        self.SubDrive = "Stop"
        self.iAction = 0
        self.LastRef = [0,0]
        self.DistFromLastRef = 0.0
        self.nPnt = 0

        # self.BatteryVoltage = 0.0
        self.enableLowBatteryMode = True
        self.lowBatteryMode = False

        self.desiredV = 0.0
        self.desiredW = 0.0
        self.currentV = 0.0
        self.currentW = 0.0
        
        self.MAXV = 100.0
        self.MAXW = 140.0 # for 12 bit resolution for analogWrite
        self.MAXWI = 20.0
        self.AutonomousMaxSpeed = 40.0
        self.SAR_Max_Speed = 25

        self.EarthRadiuskm = 6378.14
        self.navigationRadius = 2
        self.waypointRadius = 3
        
        self.P_Coeff = 6.0
        self.I_Coeff = 0.0
        self.SPEED_P_COEFF = 0.5 # this is a very first order approximation - please adjust depending on value of AutonomousMaxSpeed
        self.ROVER_POSITION_ERROR = 0.1 # again, adjust depending on desired position? 

        self.DesiredUpdateTime = datetime.datetime.now()
        self.DesiredUpdateInterval = 0.1
        self.DestinationUpdateTime = datetime.datetime.now()
        self.DestinationUpdateInterval = 0.2
        self.LowBatteryInterval = 900
        # ---Initiate Subscribers
        self.ListenToGPS = self.create_subscription(MyGPSmsg,'GPSPub_topic',self.UpdateGPS,10)
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        # ---Initiate Services
        self.server_ = self.create_service(SendString,"Driving_service",self.Driving_Server)
        # ---Initiate Clients
        self.client_Radio = self.create_client(SendString,"Radio_service")
        self.client_ApRES = self.create_client(SendString,"ApRES_service")
        self.client_WriteSpeed = self.create_client(SpeedValues,"WriteSpeed_service")
        self.client_Target_Telemetry = self.create_client(NextTarget,"Target_Telemetry_service")
        # ---Initiate Timers
        self.create_timer(0.1,self.timer_AutoDrive)
        # Initiate GPS and Telemetry timer
        self.t0GPSage = datetime.datetime.now()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def UpdateGPS(self, msg):
        try:
            d = datetime.datetime.now() - self.t0GPSage
            self.gpsAge = round(d.seconds + (d.microseconds)/1000000,1)
            self.t0GPSage = datetime.datetime.now()
            self.gpsTime = msg.time 
            self.LastPose = [msg.lat,msg.lon]
            if msg.iqual != 9:
                self.Alt = msg.alt
                self.iQual = msg.iqual
            if msg.iqual == 9:
                self.LastCourse = msg.course
            _,_,self.DistFromLastRef = self.geodesic.inv(self.LastPose[1],self.LastPose[0],self.LastRef[1],self.LastRef[0])
            self.DistFromLastRef = round(self.DistFromLastRef,2)
            if self.SurveyMode == "SAR" and self.SubDrive != "Wait":
                if self.DistFromLastRef >= self.sarSpacing-(self.sarSpacing*0.01):
                    self.Client_Write2Logger("info",f"AutoDrive: SAR --> Distance from last reference {self.DistFromLastRef}")
                    self.SurveyMode == "waitSAR"
                    self.ActionStop()
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: UpdateGPS")
            self.Client_Write2Logger("exception",str(e))
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
        try:
            self.DoubleCheckStop()
            self.iAction += 1
            self.Client_Write2Logger("info",f"AutoDrive: ActionUpdate --> iAction: {self.iAction}")
            act = self.NextTargetAct[self.iAction]
            if act == "Shoot": # ApRES Burst
                self.Client_Write2Radio(f"Rover reached to a new reference point - {self.DistFromLastRef}")
                self.Client_Write2Logger("info",f"Rover reached to a new reference point - {self.DistFromLastRef}")
                self.Client_Write2Logger("info","ActionUpdate @ Burst")
                self.Client_Write2ApRES("Burst")
                self.SubDrive = "Wait"
                self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
            elif act == "Ref": # Update reference point
                self.Client_Write2Logger("info","ActionUpdate @ Ref")
                self.LastRef = self.LastPose
                self.SurveyMode = "SAR"
                self.SubDrive = "Action"
                self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
            elif act == "Next":
                self.Client_Write2Logger("info","ActionUpdate @ Next")
                if self.SurveyMode == "SAR":
                    self.iNextTarget = -1
                else:
                    self.iNextTarget += 1
                self.iAction = 0
                self.SubDrive = "Go"
                self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
            elif act == "Done":
                self.Client_Write2Logger("info","ActionUpdate @ Done")
                self.Client_Write2Radio("Survey: Done")
                self.EmStop()
                self.SurveyMode = False
                self.SubDrive = "Stop"
                self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
            # if self.iAction == len(self.NextTargetAct):
            #     self.Client_Write2Logger("info","ActionUpdate @ All Actions Are Done")
            #     self.iAction = 0
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: ActionUpdate")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Destination_Speed_Update(self):
        try:
            if DeltaTime(self.DestinationUpdateTime) > self.DestinationUpdateInterval:
                self.DestinationUpdateTime = datetime.datetime.now()
                # update Rover's last position and course received from GPS
                self.RoverNow = self.LastPose
                self.RoverCourseNow = self.LastCourse
                # Update Rover's next target
                self.NextTargetLat = self.TargetListLat[self.iNextTarget]
                self.NextTargetLon = self.TargetListLon[self.iNextTarget]
                self.NextTargetAct = self.TargetListAct[self.iNextTarget]
                target = [self.NextTargetLat,self.NextTargetLon]
                # find distance and foraward and backward bearing to te next target from current position
                self.UpdateDist_Az(target)
                self.TELDist2Targ = self.DistToNextTarget
                self.TELBrng2Targ = self.BearingToNextTarget
                if self.DistToNextTarget < self.waypointRadius:
                    self.Client_Write2Logger("info",f"AutoDrive: Distance to the next target {self.TELDist2Targ}")
                    self.Client_Write2Radio(f"Rover reached to destination #{self.iNextTarget+1}/{self.nTargetPoints}")
                    # What should the rover do after it reached to the destination
                    if self.NextTargetAct[self.iAction] == "Stop":
                        self.Client_Write2Logger("info","Rover @ stop point")
                        if self.SurveyMode == "SAR":
                            self.SurveyMode = False
                        self.ActionStop()
                    elif self.NextTargetAct[self.iAction] == "Next":
                        self.Client_Write2Logger("info","Rover @ next point")
                        self.SubDrive = "Go"
                        self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
                        self.iNextTarget += 1
                        self.iAction = 0
                        self.DesiredSpeedUpdate()
                else:
                    self.DesiredSpeedUpdate()
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: Destination_Speed_Update")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def UpdateDist_Az(self,target):
        try:
            self.BearingToNextTarget,self.BearingFromNextTarget,self.DistToNextTarget = self.geodesic.inv(self.RoverNow[1],self.RoverNow[0],target[1],target[0])
            self.BearingToNextTarget = round(self.BearingToNextTarget,2)
            self.BearingFromNextTarget = round(self.BearingFromNextTarget,2)
            self.DistToNextTarget = round(self.DistToNextTarget,2)
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: UpdateDist_Az")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def EmStop(self):
        try:
            self.desiredW = 0.0
            self.desiredV = 0.0
            self.Client_WriteSpeed(self.desiredV,self.desiredW)
            self.DriveMode = "Manual"
            self.SurveyMode = False
            self.SubDrive = "Stop"
            self.Client_Write2Logger("info",f"AutoDrive: Emergency Stop")
            self.Client_Write2Logger("info",f"AutoDrive: driving mode set to {self.DriveMode}")
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: EmStop")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def DoubleCheckStop(self):
        try:
            self.desiredW = 0.0
            self.desiredV = 0.0
            self.Client_WriteSpeed(self.desiredV,self.desiredW)
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: DoubleCheckStop")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ActionStop(self):
        try:
            self.desiredW = 0.0
            self.desiredV = 0.0
            self.Client_WriteSpeed(self.desiredV,self.desiredW)
            if self.SurveyMode == "SAR":
                self.NextTargetAct = ["Stop","Shoot","Ref","Next"]
            self.SubDrive = "Action"
            self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: ActionStop")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def GetHeadingError(self,initial, final):
        try:
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
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: GetHeadingError")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def DesiredSpeedUpdate(self):
        try:
            IntegralHeadingError = 0.0
            headingError = 0.0
            if DeltaTime(self.DesiredUpdateTime) > self.DesiredUpdateInterval:
                self.DesiredUpdateTime = datetime.datetime.now()
                if self.gpsAge > 5:
                    self.EmStop()
                    self.Client_Write2Radio("GPS failed")
                # elif self.enableLowBatteryMode == True and self.lowBatteryMode == True:
                #     self.EmStop()
                #     self.Client_Write2Radio("Rover Stopped: low battery")
                else:
                    if self.DistToNextTarget > self.navigationRadius:
                        # convert to decimal radians
                        latitude1  = math.pi * self.NextTargetLat / 180.0
                        longitude1 = math.pi * self.NextTargetLon / 180.0
                        # implements the "sliding waypoint" or "pure pursuit" algorithm for path following
                        goalDist = self.DistToNextTarget - self.navigationRadius
                        d = goalDist / 1000.0
                        brng = math.pi * self.BearingFromNextTarget / 180.0
                        # dead reckon location "goaldist" distance from goal along path to previous goal
                        latitude2 = math.asin(math.sin(latitude1) * math.cos(d / self.EarthRadiuskm) + math.cos(latitude1) * math.sin(d / self.EarthRadiuskm) * math.cos(brng))
                        longitude2 = longitude1 + math.atan2(math.sin(brng) * math.sin(d / self.EarthRadiuskm) * math.cos(latitude1), math.cos(d / self.EarthRadiuskm) - math.sin(latitude1) * math.sin(latitude2))
                        # convert back to degrees
                        TargetLat = latitude2 * 180.0 / math.pi
                        TargetLon = longitude2 * 180.0 / math.pi
                        newTarget = [TargetLat,TargetLon]
                        self.UpdateDist_Az(newTarget)
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
                    self.desiredV = constrain(self.desiredV, 0, V_Max)
                    if self.DistToNextTarget <= self.ROVER_POSITION_ERROR:
                        self.desiredV = 0
                    self.Client_WriteSpeed(self.desiredV,self.desiredW)
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: DesiredSpeedUpdate")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def radio_listener(self, msg):
        RadioMsg = msg.data
        try:
            RadioList = RadioMsg.split(",")
            RadioCommand = RadioList[0]
            RadioSubCommand = RadioList[1:]
            #------------------------------
            if RadioCommand == "CMDDriveMode":
                if RadioSubCommand[0]== "Autonomous":
                    self.AutonomousStart(RadioSubCommand)
                self.Client_Write2Logger("info",f"AutoDrive: driving mode set to {self.DriveMode}")
                self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
            #------------------------------
            elif RadioCommand == "CMDEmStop":
                self.EmStop()
            #------------------------------
            elif RadioCommand == "UpdateHome":
                # Update Home location
                self.ThisIsHome = self.LastPose
                MsgForRadio = f"HomeUpdated,{self.ThisIsHome[0]},{self.ThisIsHome[1]}"
                self.Client_Write2Radio(MsgForRadio)
                self.Client_Write2Logger("info",f"AutoDrive: home updated {self.ThisIsHome[0]},{self.ThisIsHome[1]}")
            #------------------------------
            elif RadioCommand == "ComeHome":
                self.nTargetPoints = 1
                self.TargetListLat = [float(self.ThisIsHome[0])]
                self.TargetListLon = [float(self.ThisIsHome[1])]
                self.TargetListAct = [["Stop","Done"]]
                self.SurveyMode = "ComeHome"
                self.AutonomousStart(RadioSubCommand)
                self.Client_Write2Logger("info",f"AutoDrive: autonomous come home mode")
                self.Client_Write2Logger("info",f"AutoDrive: driving mode set to {self.DriveMode}")
                self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
            #------------------------------
            elif "Survey" in RadioCommand:
                self.ListTargetPoints(RadioCommand,RadioSubCommand)
                if RadioCommand == "AdventureSurvey":
                    self.TargetListAct = [["Next"]] * self.nTargetPoints
                    self.TargetListAct[-1] = ["Stop","Done"]
                    self.SurveyMode = "Adventure"
                    self.Client_Write2Logger("info",f"AutoDrive: {self.nTargetPoints} adventure survey points")
                #------------------------------
                elif RadioCommand == "PolarimetricSurvey":
                    self.TargetListAct = [["Stop","Shoot","Next"]] * self.nTargetPoints
                    self.TargetListAct[-1] = ["Stop","Shoot","Done"]
                    self.SurveyMode = "Polarimetric"
                    self.Client_Write2Logger("info",f"AutoDrive: {self.nTargetPoints} polarimetric survey points")
                #------------------------------
                elif RadioCommand == "SARSurvey":
                    if self.nTargetPoints == 2:
                        self.TargetListLat[0] = self.LastPose[0]
                        self.TargetListLon[0] = self.LastPose[1]
                        self.TargetListAct = [["Stop","Shoot","Ref","Next"],["Stop","Shoot","Done"]]
                    elif self.nTargetPoints == 3:
                        self.TargetListAct = [["Next"],["Stop","Shoot","Ref","Next"],["Stop","Shoot","Done"]]
                    self.SurveyMode = "SAR_0"
                    self.Client_Write2Logger("info",f"AutoDrive: {self.nTargetPoints} SAR survey points")
                #------------------------------
            elif RadioCommand == "CMDSetLowBatt":
                if RadioSubCommand[0] == "true":
                    self.enableLowBatteryMode = True
                else:   
                    self.enableLowBatteryMode = False
                self.Client_Write2Logger("info",f"AutoDrive: low battery check changed to {self.enableLowBatteryMode}")
            #------------------------------
            elif RadioCommand == "CMDAutonomousMaxSpeed":
                self.AutonomousMaxSpeed = float(RadioSubCommand[0])
                self.Client_Write2Logger("info",f"AutoDrive: new autonomous max speed {self.AutonomousMaxSpeed}")
            #------------------------------
            elif RadioCommand == "CMDSARMaxSpeed":
                self.SAR_Max_Speed = float(RadioSubCommand[0])
                self.Client_Write2Logger("info",f"AutoDrive: new autonomous SAR max speed {self.SAR_Max_Speed}")
            #------------------------------
            elif RadioCommand == "CMDSetNavRad":
                self.navigationRadius = float(RadioSubCommand[0])
                self.Client_Write2Logger("info",f"AutoDrive: new navigation radius {self.navigationRadius}")
            #------------------------------
            elif RadioCommand == "CMDSetWayPointRad":
                self.waypointRadius = float(RadioSubCommand[0])
                self.Client_Write2Logger("info",f"AutoDrive: new waypoint radius {self.waypointRadius}")
            #------------------------------
            elif RadioCommand == "CMDSetPID":
                self.P_Coeff = float(RadioSubCommand[0])
                self.I_Coeff = float(RadioSubCommand[1])
                self.SPEED_P_COEFF = float(RadioSubCommand[2])
                self.ROVER_POSITION_ERROR = float(RadioSubCommand[3])
                self.Client_Write2Logger("info",f"AutoDrive: new coeff. {self.P_Coeff},{self.I_Coeff},{self.SPEED_P_COEFF},{self.ROVER_POSITION_ERROR}")
            #------------------------------
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: radio_listener")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def AutonomousStart(self,RadioSubCommand):
        self.Client_Write2Logger("info",f"AutoDrive: ***AUTO***")
        self.EmStop()
        self.RoverNow = self.LastPose
        self.LastRef = self.LastPose
        self.iNextTarget = 0
        self.desiredV = self.AutonomousMaxSpeed #start driving forward for heading
        self.DriveMode = RadioSubCommand[0]
        self.iAction = 0
        self.SubDrive = "Go"
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ListTargetPoints(self,RadioCommand,RadioSubCommand):
        try:
            self.TargetListLat = []
            self.TargetListLon = []
            self.TargetListAct = []
            if RadioCommand == "SARSurvey":
                self.sarSpacing = float(RadioSubCommand[0])
                self.Client_Write2Logger("info",f"SAR Spacing: {self.sarSpacing}")
                self.nTargetPoints = int((len(RadioSubCommand)-1)/2)
            else:
                self.nTargetPoints = int(RadioSubCommand[0])
            LatLon = RadioSubCommand[1:]
            for i in range(self.nTargetPoints):
                ii = 2*i
                self.TargetListLat.append(LatLon[ii])
                self.TargetListLon.append(LatLon[ii+1])
            self.TargetListLat = list(map(float,self.TargetListLat))
            self.TargetListLon = list(map(float,self.TargetListLon))
            self.Client_Write2Radio(f"Target points updated ({self.nTargetPoints} points: {LatLon})")
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: ListTargetPoints")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Driving_Server(self,request, response):
        try:
            msg = request.data
            if msg == "ApRES: Done":
                self.nPnt += 1
                # self.Client_Write2Logger("info",">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                # self.Client_Write2Logger("info",f"ApRES: burst @{self.nPnt},{self.gpsTime},{self.RoverNow},{self.Alt},{self.iQual}")
                # self.Client_Write2Logger("info","<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
                self.SubDrive = "Action"
                self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
            elif msg == "ApRES: Failed": 
                self.Client_Write2Radio("Survey: Interupted")
                self.EmStop()
                self.SurveyMode = False
                self.SubDrive = "Stop"
                self.Client_Write2Logger("info",f"AutoDrive: sub-drive mode set to {self.SubDrive}")
            response.msg_sent = True
            return response
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: Driving_Server")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Target2Telemetry(self):
        try:
            request = NextTarget.Request()
            request.i_next_target = self.iNextTarget
            request.next_target_lat = self.NextTargetLat
            request.next_target_lon = self.NextTargetLon
            request.bearing_to_next_target = self.TELBrng2Targ
            request.dist_to_next_target = self.TELDist2Targ 
            request.dist_from_last_ref = self.DistFromLastRef
            self.client_Target_Telemetry.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: Client_Target2Telemetry")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_WriteSpeed(self,desiredV,desiredW):
        try:
            request = SpeedValues.Request()
            request.v_linear = float(desiredV)
            request.w_angular = float(desiredW)
            self.client_WriteSpeed.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: Client_WriteSpeed")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        try:
            request = SendString.Request()
            request.data = MsgForRadio
            self.client_Radio.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: Client_Write2Radio")
            self.Client_Write2Logger("exception",str(e))
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2ApRES(self,MsgForApRES):
        try:
            request = SendString.Request()
            request.data = MsgForApRES
            self.client_ApRES.call_async(request)
        except Exception as e:
            self.Client_Write2Logger("error","AutoDrive: Client_Write2ApRES")
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
    node = AutoDrive() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
