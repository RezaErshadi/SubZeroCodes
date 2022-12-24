#include "Arduino.h"
#include "SpeedControlRoboteq.h"
#include <Servo.h>

//define the different motor pins
//left
const byte BL = 12;
const byte FR = 6;

//right
const byte FL =  8;
const byte BR =  10 ;





SpeedControlRoboteq::SpeedControlRoboteq()
{
	pinMode(FL, OUTPUT);
	pinMode(BL, OUTPUT);  
	pinMode(FR, OUTPUT);
	pinMode(BR, OUTPUT);  
	analogWriteResolution(12);
}

void SpeedControlRoboteq::SpeedSet(float v, float w)
{	
			
	v = ((v*1718/100) + 2055); //new 12 bit math	
	
	v = constrain(v,321,4079); //new 12 bit math
	
	int vprime = (int)v;
	int wprime = (int)w;
		
	if(isnan(vprime)){
		vprime = 0;
	}
	if(isnan(wprime)){
		wprime = 0;
	}


	analogWrite(FL, constrain(vprime + wprime,321,3773));	
	analogWrite(BL, constrain(vprime + wprime,321,3773));			
	analogWrite(FR, constrain(vprime - wprime,312,3773));	
	analogWrite(BR, constrain(vprime - wprime,321,3773));	
}


