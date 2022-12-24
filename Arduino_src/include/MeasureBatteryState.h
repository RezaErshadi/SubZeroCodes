// Include Arduino framework
#include <Arduino.h>

const unsigned long BatteryInterval = 1000;
const unsigned long LowBatteryInterval = 900000; //15 minutes
float batteryVoltage = 0.0;
bool lowBatteryMode = false;
bool enableLowBatteryMode = true;
#define BatteryPin A4 

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
        // Switch to lowBatteryMode if the voltage drops below 49
        if (!lowBatteryMode && batteryVoltage < 49.0) {
            lowBatteryMode = true;
            LowBatteryTime = millis();
        }
        // Turn off the lowBatteryMode if the voltage is above 51
        else if (lowBatteryMode && batteryVoltage > 51.0 && millis() - LowBatteryTime > LowBatteryInterval) {
            lowBatteryMode = false;
        }
    }
}