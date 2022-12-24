#!/usr/bin/env python3
"""
This node opens a serial port for Radio communication
It always listen to the messages coming from the radio and publishes them via:
topic_MsgFromRadio
in a form of comma seperated messages and main command and sub commands
It also listens to any messages from other nodes which must be sent into the radio via:
topic_MsgToRadio
"""
from click import echo_via_pager
import rclpy # ROS2 Python library
from rclpy.node import Node # import the Node module
# Interfaces
from std_msgs.msg import String
from interfaces_subzero.srv import SendString
from interfaces_subzero.msg import MyGPSmsg
from interfaces_subzero.srv import ApRESConf
# Python packages
import time
import datetime
import pyproj
import math

"""
float 		robotLength 	= 1.05;
float	  	robotWidth		= 1.20; //width between center of wheels on same axle
"""

class Driving(Node): # MODIFY NAME
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def __init__(self):
        # ---Initiate the Node object
        super().__init__('Driving')
        # ---Initiate Parameters
        self.geodesic = pyproj.Geod(ellps='WGS84')
        self.RadioPingTimer = datetime.datetime.now()

        self.SurveyMode = False
        self.DriveMode = "Manual"

        self.BatteryVoltage = 0

        self.desiredV = 0.0
        self.desiredW = 0.0
        self.currentV = 0.0
        self.currentW = 0.0
        self.LinearIncrement = 10
        self.DifferentialIncrement = 28.0
        self.MAXV = 100.0
        self.MAXW = 140.0 # for 12 bit resolution for analogWrite
        self.MAXWI = 20.0
        self.AutonomousMaxSpeed = 45.0
        self.EarthRadiuskm = 6378.14
        self.waypointRadius = 25
        self.navigationRadius = 15
        self.P_Coeff = 6.0
        self.I_Coeff = 0.0
        self.D_Coeff = 0.0
        self.A_Coeff = -2.0
        self.Woffset = 0.0
        self.Wfactor = 1.0
        self.Ffactor = 1717
        self.upperlimit = 3777
        self.lowerlimit = 335
        self.cent = 2056

        self.TelemetryInterval = 5
        self.DesiredUpdateInterval = 0.1

        self.IgnoreSerial = False
        self.enableLowBatteryMode = True
        self.lowBatteryMode = False

        self.TargetListLat = []
        self.TargetListLon = []
        self.TargetListAct = []
        self.nTargetPoints = 0
        self.iNextTarget = 0
        self.NextTargetLat = 0
        self.NextTargetLon = 0
        self.NextTargetAct = ""
        self.iAction = 0
        self.DistToNextTarget = 0
        self.BearingToNextTarget = 0
        self.ActionRequiredFlag = False
        self.TELDistToNextTarget = 0
        self.TELBearingToNextTarget = 0

        # ---Initiate Subscribers
        self.ListenToGPS = self.create_subscription(MyGPSmsg,'GPSPub_topic',self.UpdateGPS,10)
        self.ListenToRadio = self.create_subscription(String,'RadioPub_topic',self.radio_listener,10)
        self.ListenToArduino = self.create_subscription(String,'ArduinoPub_topic',self.arduino_listener,10)
        # ---Initiate Services
        self.server_ = self.create_service(SendString,"Driving_service",self.Driving_Server)
        # ---Initiate Clients
        self._client_Radio = self.create_client(SendString,"Radio_service")
        self._client_Arduino = self.create_client(SendString,"Arduino_service")
        self._client_ApRES = self.create_client(ApRESConf,"ApRES_service")
        # ---Initiate Timers
        self.create_timer(0.5,self.timer_RadioRangeCheck)
        self.create_timer(0.13,self.timer_AutoDrive)
        self.create_timer(0.13,self.timer_ActionInPlace)
        self.create_timer(1,self.timer_TransmitTelemetry)
        # Initiate GPS and Telemetry timer
        time.sleep(1)
        self.t0GPSage = datetime.datetime.now()
        self.t0Telemetry = datetime.datetime.now()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def timer_AutoDrive(self):
        if (self.DriveMode == "Autonomous") and (self.nTargetPoints > 0):
            self.DestinationUpdate()
            self.DesiredSpeedUpdate()
            self.ActualSpeedUpdate()
        else:
            pass
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def timer_ActionInPlace(self):
        if self.ActionRequiredFlag == True:
            self.EmStop()
            act = self.NextTargetAct[self.iAction]
            self.iAction += 1
            if act == "Done":
                time.sleep(1)
                self.Client_Write2Radio("Survey: Done")
                self.EmStop()
                self.ActionRequiredFlag = False
            elif act == "Shoot":
                self.EmStop()
                self.ActionRequiredFlag = False
                self.Client_Write2ApRES("Burst")
            elif act == "Ref":
                self.EmStop()
                self.RefPoint = self.RoverNow
                self.SurveyMode = "SAR"
                self.ActionRequiredFlag = True
            elif act == "SarNext":
                self.iNextTarget = -1
                self.NextTargetLat = self.TargetListLat[self.iNextTarget]
                self.NextTargetLon = self.TargetListLon[self.iNextTarget]
                self.NextTargetAct = self.TargetListAct[self.iNextTarget] 
                self.ActionRequiredFlag = False
                self.DriveMode = "Autonomous"
            elif act == "Next":
                self.iNextTarget += 1
                self.ActionRequiredFlag = False
                self.DriveMode = "Autonomous"
            if self.iAction == len(self.NextTargetAct):
                self.iAction = 0
                self.ActionRequiredFlag = False
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def DestinationUpdate(self):
        self.NextTargetLat = self.TargetListLat[self.iNextTarget]
        self.NextTargetLon = self.TargetListLon[self.iNextTarget]
        self.NextTargetAct = self.TargetListAct[self.iNextTarget] 
        # find distance and foraward and backward bearing to te next target from current position
        self.BearingToNextTarget,self.back_azimuth,self.DistToNextTarget = self.geodesic.inv(self.RoverNow[1],self.RoverNow[0],self.NextTargetLon,self.NextTargetLat)
        _,_,self.DistFromRef = self.geodesic.inv(self.RoverNow[1],self.RoverNow[0],self.RefPoint[1],self.RefPoint[0])
        self.BearingToNextTarget = round(self.BearingToNextTarget,2)
        self.back_azimuth = round(self.back_azimuth,2)
        self.DistToNextTarget = round(self.DistToNextTarget,2)
        self.DistFromRef = round(self.DistFromRef,2)
        self.TELDistToNextTarget = self.DistToNextTarget
        self.TELBearingToNextTarget = self.BearingToNextTarget
        self.get_logger().info(f"Distance from Ref: {self.DistFromRef}")
        if self.DistToNextTarget < self.waypointRadius:
            self.Client_Write2Radio(f"Rover reached to point #{self.iNextTarget+1}/{self.nTargetPoints}")
            # What should the rover do after it reached to the destination
            if self.NextTargetAct[self.iAction] == "Stop":
                self.ActionRequiredFlag = True
                self.iAction += 1
                self.EmStop()
            elif self.NextTargetAct[self.iAction] == "Next":
                self.ActionRequiredFlag = False
                self.iNextTarget += 1
        elif (self.SurveyMode == "SAR") and (self.DistFromRef >= self.sarSpacing-(self.sarSpacing*0.05)):
            self.ActionRequiredFlag = True
            self.NextTargetAct = ["Stop","Shoot","Ref","SarNext"]
            self.iAction = 1
            self.EmStop()
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
        angle = 0 # we fixed the front panel
        if self.gpsAge > 5000:
                self.Stop()
                self.Client_Write2Radio("GPS failed")
        elif (self.enableLowBatteryMode == True) and (self.lowBatteryMode):
            self.desiredW = 0.0
            self.desiredV = 0.0
        else:
            if self.DistToNextTarget > self.navigationRadius:
                # implements the "sliding waypoint" or "pure pursuit" algorithm for path following
                goalDist = self.DistToNextTarget - self.navigationRadius
                d = goalDist / 1000.0
                # convert to decimal radians
                latitude1  = math.pi * self.NextTargetLat / 180.0
                longitude1 = math.pi * self.NextTargetLon / 180.0
                brng = math.pi * self.back_azimuth / 180.0
                # dead reckon location "goaldist" distance from goal along path to previous goal
                latitude2 = math.asin(math.sin(latitude1) * math.cos(d / self.EarthRadiuskm) + math.cos(latitude1) * math.sin(d / self.EarthRadiuskm) * math.cos(brng))
                longitude2 = longitude1 + math.atan2(math.sin(brng) * math.sin(d / self.EarthRadiuskm) * math.cos(latitude1), math.cos(d / self.EarthRadiuskm) - math.sin(latitude1) * math.sin(latitude2))
                # convert back to degrees
                TargetLat = latitude2 * 180.0 / math.pi
                targetLNG = longitude2 * 180.0 / math.pi
                # calculate bearing to new target point
                self.BearingToNextTarget,self.back_azimuth,self.DistToNextTarget = self.geodesic.inv(self.RoverNow[1],self.RoverNow[0],targetLNG, TargetLat)
                self.BearingToNextTarget = round(self.BearingToNextTarget,2)
                self.back_azimuth = round(self.back_azimuth,2)
                self.DistToNextTarget = round(self.DistToNextTarget,2)
        # calculate the error between actual heading and bearing to goal
        headingError = self.GetHeadingError(self.RoverCourseNow, self.BearingToNextTarget)
        # calculate integral term
        IntegralHeadingError = IntegralHeadingError + ((self.DesiredUpdateInterval / 1000.0) * headingError)
        # prevents dividing by zero for I_Coeff = 0
        if self.I_Coeff == 0:
            IntegralHeadingError = 0.0
        # constrain to prevent windup
        else:
            IntegralHeadingError = self.constrain(IntegralHeadingError, -self.MAXWI/self.I_Coeff,self.MAXWI/self.I_Coeff)
        pivotUse = 0.0
        if abs(headingError) < 25:
            pivotUse = 1.0
        # set desired V and W
        self.desiredW = self.P_Coeff * headingError + self.I_Coeff * IntegralHeadingError + self.A_Coeff*angle*pivotUse
        self.desiredW = self.constrain(self.desiredW, -self.MAXW, self.MAXW) # ensure that 'abs(desiredW) < MAXW'
        self.desiredV = self.AutonomousMaxSpeed
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ActualSpeedUpdate(self):
        # constrain W if V too low
        if abs(self.currentV) < 10:
            self.desiredW = 0.0
        else:
            self.desiredW = self.constrain(self.desiredW,(-abs(self.currentV)*16 + 10.0),(abs(self.currentV)*16 - 10.0))
        newV = self.currentV
        newW = self.currentW   
        vDiff = self.desiredV - self.currentV
        wDiff = self.desiredW - self.currentW   
        signvDiff = 1
        signwDiff = 1
        if(vDiff < 0): 
            signvDiff = -1
        if(wDiff < 0): 
            signwDiff = -1
        # define and constrain the new V and W values
        newV = self.currentV + (signvDiff * min(self.LinearIncrement,abs(vDiff)))
        newW = self.currentW + (signwDiff * min(self.DifferentialIncrement,abs(wDiff)))
        # define and constrain the new V and W values    
        self.currentV = self.constrain(newV, -self.MAXV, self.MAXV)	
        self.currentW = self.constrain(newW, -self.MAXW, self.MAXW)
        self.WriteSpeed()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def WriteSpeed(self):
        if (self.currentV>=0):
            vpFL = ((((self.currentV*80)/100)*self.Ffactor)/100)+self.cent
        else:
            vpFL = ((((self.currentV*83)/100)*self.Ffactor)/100)+self.cent
        vpBL = ((self.currentV*self.Ffactor)/100)+self.cent
        vpFR = ((self.currentV*self.Ffactor)/100)+self.cent
        vpBR = ((self.currentV*self.Ffactor)/100)+self.cent
        wPrime = self.Wfactor * (self.currentW + self.Woffset)
        vFL = self.constrain(vpFL + wPrime,self.lowerlimit,self.upperlimit)
        vBL = self.constrain(vpBL + wPrime,self.lowerlimit,self.upperlimit)
        vFR = self.constrain(vpFR - wPrime,self.lowerlimit,self.upperlimit)
        vBR = self.constrain(vpBR - wPrime,self.lowerlimit,self.upperlimit)
        # write actual values to output registers --> to motor controllers
        WriteVelocity = f"CMDSetSpeed,{vFL},{vBL},{vFR},{vBR}"
        self.Client_Write2Arduino(WriteVelocity)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Stop(self):
        if self.DriveMode == "Manual":
            self.currentV = 0
            self.currentW = 0
            self.WriteSpeed()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def EmStop(self):
            self.DriveMode = "Manual"
            self.currentV = 0
            self.currentW = 0
            self.WriteSpeed()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def constrain(self,val,lowerlimit,upperlimit):
        if val < lowerlimit:
            val = lowerlimit
        elif val > upperlimit:
            val = upperlimit
        return val
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def arduino_listener(self, msg):
        ArduinoMsg = msg.data
        self.ArduinoList = ArduinoMsg.split(",")
        self.ArduinoCommand = self.ArduinoList[0]
        self.ArduinoSubCommand = self.ArduinoList[1:]
        if self.ArduinoCommand == "TELArduino":
            self.BatteryVoltage = float(self.ArduinoSubCommand[0])
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def radio_listener(self, msg):
        RadioMsg = msg.data
        self.RadioPingTimer = datetime.datetime.now()
        self.RadioList = RadioMsg.split(",")
        self.RadioCommand = self.RadioList[0]
        self.RadioSubCommand = self.RadioList[1:]
        if self.RadioCommand == "UpdateHome":
            # "UpdateHome"
            self.ThisIsHome = self.RoverNow
            #------------------------------ 
        elif "Survey" in self.RadioCommand:
            if self.RadioCommand == "GoThereSurvey":
                self.GoThere()
                #------------------------------
            elif self.RadioCommand == "AdventureSurvey":
                self.ReadAdventurePoints()
                #------------------------------
            elif self.RadioCommand == "PolarimetricSurvey":
                self.ReadPolarimetricPoints()
                #------------------------------
            elif self.RadioCommand == "SARSurvey":
                self.ReadSARpoints()
                #------------------------------
            self.get_logger().info(f"{self.nTargetPoints},{self.TargetListLat},{self.TargetListLon},{self.TargetListAct}")
        elif self.RadioCommand == "ComeHome":
            self.ComeHome()
        elif self.RadioCommand == "ApRES":
            if self.RadioSubCommand[0] == "Burst" and self.DriveMode == "Manual":
                self.Stop()
                time.sleep(0.5) # make sure the rover is stopped
                self.ActionRequiredFlag = False
                self.Client_Write2ApRES(["Burst"])
        elif self.RadioCommand == "CMDDriveMode":
            if self.RadioSubCommand[0]== "Autonomous":
                self.EmStop()
                self.RefPoint = self.RoverNow
                self.iNextTarget = 0
                self.desiredV = self.AutonomousMaxSpeed #start driving forward for heading
                self.ActionRequiredFlag = False
                self.DriveMode = self.RadioSubCommand[0]
        elif self.RadioCommand == "CMDSetSpeed":
            self.desiredV = float(self.RadioSubCommand[0])
            self.desiredW = float(self.RadioSubCommand[1])
            self.ActualSpeedUpdate()
        elif self.RadioCommand == "CMDStop":
            self.Stop()
        elif self.RadioCommand == "CMDEmStop":
            self.EmStop()
        elif self.RadioCommand == "CMDSetLowBatt":
            self.Client_Write2Arduino(RadioMsg)
            if self.RadioSubCommand[0] == "true":
                self.enableLowBatteryMode = True
            else:
                self.enableLowBatteryMode = False
        elif self.RadioCommand == "CMDAutonomousMaxSpeed":
            self.AutonomousMaxSpeed = float(self.RadioSubCommand[0])
        elif self.RadioCommand == "CMDSetNavRad":
            self.navigationRadius = float(self.RadioSubCommand[0])
        elif self.RadioCommand == "CMDSetWOffset":
            self.Woffset = float(self.RadioSubCommand[0])
        elif self.RadioCommand == "CMDSettingsQuery":
            pass
        elif self.RadioCommand == "CMDSetWayPointRad":
            self.waypointRadius = float(self.RadioSubCommand[0])
        elif self.RadioCommand == "CMDSetPID":
            self.P_Coeff = float(self.RadioSubCommand[0])
            self.I_Coeff = float(self.RadioSubCommand[1])
            self.A_Coeff = float(self.RadioSubCommand[2])
            self.D_Coeff = float(self.RadioSubCommand[3])
        elif self.RadioCommand == "CMDSetIgnoreInstrument":
            self.Client_Write2Arduino(RadioMsg)
            self.IgnoreSerial = self.RadioSubCommand[0] #ignore/listen
        elif self.RadioCommand == "CMDSetTelemetryInterval":
            self.Client_Write2Arduino(RadioMsg)
            self.TelemetryInterval = float(self.RadioSubCommand[0])/1000 # convert to seconds
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def timer_TransmitTelemetry(self):
        d = datetime.datetime.now() - self.t0Telemetry
        dTel = round(d.seconds + (d.microseconds)/1000000,1)
        if dTel >= self.TelemetryInterval:
            TelMsg = f"TELEMETRY,{self.GPSTEL},{self.BatteryVoltage},{self.currentV},{self.currentW},{self.TELBearingToNextTarget},{self.TELDistToNextTarget},{self.iNextTarget},{self.NextTargetLat},{self.NextTargetLon}"
            self.Client_Write2Radio(TelMsg)
            self.t0Telemetry = datetime.datetime.now()
            self.get_logger().info(f"Telemetry: {TelMsg}")
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ComeHome(self):
        self.EmStop()
        self.nTargetPoints = 1
        self.TargetListLat = [float(self.ThisIsHome[0])]
        self.TargetListLon = [float(self.ThisIsHome[1])]
        self.TargetListAct = [["Stop","Done"]]
        self.RefPoint = self.RoverNow
        self.iNextTarget = 0
        self.desiredV = self.AutonomousMaxSpeed
        self.DriveMode = "Autonomous"
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ListTargetPoints(self):
        self.TargetListLat = []
        self.TargetListLon = []
        self.TargetListAct = []
        if self.RadioCommand == "SARSurvey":
            self.sarSpacing = float(self.RadioSubCommand[0])
            self.get_logger().info(f"SAR Spacing: {self.sarSpacing}")
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
    def GoThere(self):
        self.ListTargetPoints()
        self.TargetListAct = [["Stop","Done"]]
        self.SurveyMode = "GoThere"
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ReadAdventurePoints(self):
        self.ListTargetPoints()
        self.TargetListAct = [["Next"]] * self.nTargetPoints
        self.TargetListAct[-1] = ["Stop","Done"]
        self.SurveyMode = "Adventure"
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ReadPolarimetricPoints(self):
        self.ListTargetPoints()
        self.TargetListAct = [["Stop","Shoot","Next"]] * self.nTargetPoints
        self.TargetListAct[-1] = ["Stop","Shoot","Done"]
        self.SurveyMode = "Polarimetric"
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def ReadSARpoints(self):
        self.ListTargetPoints()
        self.TargetListAct = [["Next"],["Stop","Shoot","Ref","SarNext"],["Stop","Shoot","Done"]]
        self.SurveyMode = "SAR_0"
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def timer_RadioRangeCheck(self):
        # if the rover is in manual mode
        # and there is no message (including ping) from the C.S.
        # the rover will stop
        d = datetime.datetime.now() - self.RadioPingTimer
        dT = round(d.seconds + (d.microseconds)/1000000,1)
        if (dT > 3) and (self.DriveMode == "Manual") and (abs(self.desiredV) > 0):
            self.Stop()
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Driving_Server(self,request, response):
        msg = request.data
        if msg == "ApRES: Done":
            self.ActionRequiredFlag = True
        response.msg_sent = True
        return response
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Radio(self,MsgForRadio):
        request = SendString.Request()
        request.data = MsgForRadio
        self._client_Radio.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2Arduino(self,MsgForArduino):
        request = SendString.Request()
        request.data = MsgForArduino
        self._client_Arduino.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def Client_Write2ApRES(self,MsgForApRES):
        request = ApRESConf.Request()
        request.act = MsgForApRES[0]
        self._client_ApRES.call_async(request)
    """ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
    def UpdateGPS(self, msg):
        # calculate gpsAge
        d = datetime.datetime.now() - self.t0GPSage
        self.gpsAge = round(d.seconds + (d.microseconds)/1000000,1)
        self.t0GPSage = datetime.datetime.now()
        # update time from the gps
        self.gpsTime = msg.time 
        # update rover position
        self.RoverNow = [msg.lat,msg.lon]
        try:
            _,_,self.DistFromRef = self.geodesic.inv(self.RoverNow[1],self.RoverNow[0],self.RefPoint[1],self.RefPoint[0])
        except:
            self.RefPoint = self.RoverNow
        if (self.SurveyMode == "SAR") and (self.DistFromRef >= self.sarSpacing-(self.sarSpacing*0.05)):
            self.ActionRequiredFlag = True
            self.NextTargetAct = ["Stop","Shoot","Ref","SarNext"]
            self.iAction = 1
            self.EmStop()
        self.RoverCourseNow = msg.course
        self.GPSTEL = f"{msg.time},{self.gpsAge},{msg.lat},{msg.lon},{msg.alt},{msg.speed},{msg.course}"
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
def main(args=None):
    rclpy.init(args=args) # initialize ROS communications
    node = Driving() # create (Instantiate) the node (Modify the name)
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown() # shuting down the node after killing the node (ctrl+c)
""" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ """
if __name__ == "__main__":
    main()
