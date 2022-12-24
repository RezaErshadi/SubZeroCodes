#include "Arduino.h"
#include "PivotAngle.h"


const int		  stringPotLeft 	= A2;
const int		  stringPotRight 	= A0;


PivotAngle::PivotAngle()
{
	
}

double PivotAngle::MeasureAngle()
{	
		
	double leftValue = (double)analogRead(stringPotLeft);
	double rightValue = (double)analogRead(stringPotRight);
	
	double leftAngle = 108.2 + leftValue*(-0.0087) + pow(leftValue,2)*(5.2354E-7);
	double rightAngle = 73.0914 + rightValue*(0.0068) + pow(rightValue,2)*(-1.4719E-7);
		
	//Serial.print(leftAngle);
	//Serial.print(",");
	//Serial.println(rightAngle);
	return -((leftAngle + rightAngle)/2 - 90);
	
}


