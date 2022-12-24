

#ifndef SpeedControlRoboteq_h
#define SpeedControlRoboteq_h

#include "Arduino.h"
#include <Servo.h>

class SpeedControlRoboteq
{
  public:
    SpeedControlRoboteq();
    void SpeedSet(float v, float w);    
  private:
    int vprime;
	int wprime;
};

#endif