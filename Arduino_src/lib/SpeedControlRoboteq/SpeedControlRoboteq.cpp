// Version: Reza Ershadi
#include "Arduino.h"
#include "SpeedControlRoboteq.h"
#include <Servo.h>
//define the different motor pins
//left wheels
const byte BL = 12;
const byte FR = 6;
//right wheels
const byte FL =  8;
const byte BR =  10;

int vFL;
int vRL;
int vFR;
int vRR;

SpeedControlRoboteq::SpeedControlRoboteq()
{
	pinMode(FL, OUTPUT); // Front Left
	pinMode(BL, OUTPUT); // Back Left
	pinMode(FR, OUTPUT); // Front Right
	pinMode(BR, OUTPUT); // Back Right
	analogWriteResolution(12); // set the pin resolution to 12
}

void SpeedControlRoboteq::SpeedSet(float v, float w)
{		
	vFL = (int)((((v*85)/100)*1723)/100)+2056;
	vRL = (int)(((v)*1723)/100)+2056;

	vFR = (int)(((v)*1723)/100)+2056;
	vRR = (int)(((v)*1723)/100)+2056;

	int wprime = (int)w;

	analogWrite(FL, constrain(vFL + wprime,335,3777));	
	analogWrite(BL, constrain(vRL + wprime,335,3777));			
	analogWrite(FR, constrain(vFR - wprime,335,3777));	
	analogWrite(BR, constrain(vRR - wprime,335,3777));	
}


