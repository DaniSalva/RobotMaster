#pragma config(Sensor, S2,     SONAR,          sensorSONAR)
#pragma config(Sensor, S4,     HTGYRO,         sensorAnalogInactive)
#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)
#pragma config(Sensor, S1,     cam,                 sensorI2CCustomFastSkipStates)

#include "drivers/lego-light.h"
#include "mapLib.c"
#include "drivers/mindsensors-nxtcam.h"

#define SONAR_T 24

void fordwardSonar(float distance,int sonarEnabled) {
	time1[T1] = 0;
	setSpeed(0.1,0);
	int sonarValue=100;
	while(sonarValue>=SONAR_T && time1[T1]<distance*10000){
		if(sonarEnabled){
			sonarValue=SensorValue(SONAR);
		}
	}
	setSpeed(0,0);
}

task main(){
	fordwardSonar(0.4,1);
}
