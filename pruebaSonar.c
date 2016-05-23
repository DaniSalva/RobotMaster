#pragma config(Sensor, S3,     SONAR,          sensorSONAR)
#pragma config(Sensor, S4,     HTGYRO,         sensorAnalogInactive)
#pragma config(Sensor, S2,     lightSensor,         sensorLightActive)
#pragma config(Sensor, S1,     cam,                 sensorI2CCustomFastSkipStates)

#include "drivers/lego-light.h"
#include "mapLib.c"
#include "drivers/mindsensors-nxtcam.h"

task main(){
	fordwardSonar(0.4,1);
}
