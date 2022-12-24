// Include Arduino framework
#include <Arduino.h>
// Include other libraries
#include <TinyGPS++.h>
//#include <SpeedControlRoboteq.h>
#include <PivotAngle.h>
//========================================================================//
int cent = 2056;
//========================================================================//
//Battery state information
float batteryVoltage = 0.0;
//========================================================================//
//setting to ignore the instrument radio
bool IgnoreSerial = false;
//========================================================================//
//strings for radio commandss
const char Initiator = '#';
const char Terminator = '$';
//========================================================================//
int		  	stringPotLeft 	= A2;
int		  	stringPotRight 	= A0;
PivotAngle 	PivotObject;
float 		pivotAngle		= 0;
double 		angle 			= 0;
//========================================================================//
float TEL_T0 = millis();
float Check_T0 = millis();
unsigned long 		TelemetryInterval = 5000;
const unsigned long BatteryInterval = 1000;
const unsigned long PivotAngleUpdateInterval = 100;
const unsigned long LowBatteryInterval = 900000; //15 minutes
//baud rates
static const uint32_t DEBUGBaud = 38400;
//========================================================================//
//define the different motor pins
float vpFL;
float vpBL;
float vpFR;
float vpBR;
float wPrime;
const byte FL =  8; // Front Left
const byte BL = 12; // Back Left
const byte FR = 6; // Front Right
const byte BR =  10 ; // Back Right
int vFL = 2056;
int vBL = 2056;
int vFR = 2056;
int vBR = 2056;
//========================================================================//
// USB port
#define SerialUSB Serial
// A4 pin for battery voltage
#define BatteryPin A4
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void MeasureBatteryState() {
    // millis: Returns the number of milliseconds passed since the Arduino board began running the current program. 
    // This number will overflow (go back to zero), after approximately 50 days.
    static unsigned long BatteryTime = millis(); //used to measure elapsed time
    static unsigned long LowBatteryTime;
    if (millis() - BatteryTime > BatteryInterval) {
        BatteryTime = millis();
        batteryVoltage = 0;
        // read the battery pin value 10 times and average it to measure the battery voltage
        for (int i = 0; i < 10; i++) {
            batteryVoltage = batteryVoltage + analogRead(BatteryPin) * 57 / 3785.0;
        }
        batteryVoltage = batteryVoltage / 10.0;
    }
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void WriteSpeed() {
	analogWrite(FL, int(vFL));
	analogWrite(BL, int(vBL));
	analogWrite(FR, int(vFR));
	analogWrite(BR, int(vBR));
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void SetNewManualSpeed(String CommandSubString,HardwareSerial &refSer) {
    int Index1 = CommandSubString.indexOf(',');
    int Index2 = CommandSubString.indexOf(',',Index1+1);
    int Index3 = CommandSubString.indexOf(',',Index2+1);
    String StringFL = CommandSubString.substring(0, Index1);
    String StringBL = CommandSubString.substring(Index1+1, Index2);
    String StringFR = CommandSubString.substring(Index2+1, Index3);
    String StringBR = CommandSubString.substring(Index3+1);
    vFL = StringFL.toInt();
    vBL = StringBL.toInt();
    vFR = StringFR.toInt();
    vBR = StringBR.toInt();
    WriteSpeed();
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void stop(HardwareSerial &refSer) {
    vFL = cent;
    vBL = cent;
    vFR = cent;
    vBR = cent;
    WriteSpeed();
    refSer.println("#RoverStopped$");
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void PivotAngleUpdate(){
	static unsigned long PivotAngleUpdateTime = millis();	
	if(millis() - PivotAngleUpdateTime > PivotAngleUpdateInterval){
		angle = PivotObject.MeasureAngle();
        // Serial.print("Pivot ANgle: ");
        // Serial.println(angle);
        // delay(1000);
	}
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void TransmitBattery(HardwareSerial &refSer) {
        refSer.print(batteryVoltage,2);
        refSer.print(",");
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void TransmitVelocity(HardwareSerial &refSer) {
        refSer.print(vFL,1);
        refSer.print(",");
        refSer.print(vBL,1);
        refSer.print(",");
        refSer.print(vFR,1);
        refSer.print(","); 
        refSer.print(vBR,1);
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void TransmitTelemetry(HardwareSerial &refSer) {
        refSer.print("#TELArduino,");
        TransmitBattery(refSer);
        TransmitVelocity(refSer);
        refSer.println("$");
        delay(20); // wait a bit
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void RecNewMsg(HardwareSerial &refSer) { // Get new message from a computer or radio
    static String NewMsg;
    if (refSer.available() > 0) {
        refSer.readStringUntil(Initiator); // get read of everything before the initiator
        NewMsg = refSer.readStringUntil(Terminator); // read until the terminator
        // SerialUSB.print("Arduino just received --> ");
        // SerialUSB.println(NewMsg); // print the new message
        delay(20); // wait a bit
        // ------------------------------------------- Check the command
        if (refSer.read() == Terminator) {
            int Index0 = NewMsg.indexOf(','); // split the message by comma
            String CommandName = NewMsg.substring(0, Index0); // read the command name
            String CommandSubString = NewMsg.substring(Index0 + 1); // read the command substring
            if (CommandName == "CMDSetSpeed") {
                SetNewManualSpeed(CommandSubString,refSer);
            }
            if (CommandName == "CMDChecking") {
                Check_T0 = millis();
            }
            else if (CommandName == "CMDEmStop") {
                stop(refSer);
            }
            else if (CommandName == "CMDStop") {
                stop(refSer);
            }
            else if (CommandName == "CMDSetIgnoreInstrument") {
                if (CommandSubString == "ignore") {
                    IgnoreSerial = true;
                }
                else if (CommandSubString == "listen") {
                    IgnoreSerial = false;
                }
                else {
                    refSer.println("#ERRSetIgnoreInstrument,invalid$");
                    delay(20); // wait a bit
                }
            }
            else if (CommandName == "CMDSetTelemetryInterval") {
                int newTelemetryInterval = CommandSubString.toInt();
                if (newTelemetryInterval > 0 && newTelemetryInterval < 600000) {
                    TelemetryInterval = newTelemetryInterval;
                    refSer.println("#RESTelemetrySet,valid$");
                    delay(20); // wait a bit
                }
                else refSer.println("#RESTelemetrySet,invalid$");
                delay(20); // wait a bit
            }
        }
    }
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void setup() {
    pinMode(BatteryPin, INPUT);

	pinMode(FL, OUTPUT); // Front Left
	pinMode(BL, OUTPUT); // Back Left
	pinMode(FR, OUTPUT); // Front Right
	pinMode(BR, OUTPUT); // Back Right

	analogWriteResolution(12); // set the pin resolution to 12
    analogReadResolution(12); //set 12 bit resolution

    SerialUSB.begin(DEBUGBaud);

    // Set the maximum timeout to 25 ms for each Serial port.
    Serial.setTimeout(25);
    //	wait until Serial ports have initialized
    while (!Serial) {
    ;
    }
    SerialUSB.println("#Arduino Initialized$");
    delay(20); // wait a bit
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
void loop() {
    RecNewMsg(SerialUSB);
    WriteSpeed();  		//update actual speeds based on desired
    float TEL_Passed = millis() - TEL_T0;
    // float Check_Passed = millis() - Check_T0;
    // if ( Check_Passed > 17000){
    //     stop(SerialUSB);
    // }
    if (!IgnoreSerial) {
        if ( TEL_Passed > TelemetryInterval){
            MeasureBatteryState();
            TransmitTelemetry(SerialUSB); //send info back via drive radio
            TEL_T0 = millis();
        }
    }
}
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//
//========================================================================//