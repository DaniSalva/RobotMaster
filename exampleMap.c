#pragma config(Sensor, S2,     SONAR,          sensorSONAR)
#pragma config(Sensor, S4,     HTGYRO,         sensorAnalogInactive)
#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)
#pragma config(Sensor, S1,     cam,                 sensorI2CCustomFastSkipStates)

#include "drivers/lego-light.h"
#include "mapLib.c"
#include "drivers/mindsensors-nxtcam.h"

// CONFIG camera color position
#define GREEN 1
#define BLUE 2
#define RED 0

// CONFIG GOAL PARAMETERS
#define GOAL_COLOR RED
#define AREA_COLOR 100


int x_left=4;
int y_left=6;
int x_right=5;
int y_right=6;

float MIDDLE_X_CAMERA = 95.5;
float DESIRED_AREA = 2500;

float alignToBall(float x)
{
	return ((MIDDLE_X_CAMERA-x)/75);
}

float speedToBall(float area)
{
	return 0.2;

}

task main(){

  Delete(sFileName, nIoResult);
  hFileHandle = 0;

  OpenWrite(  hFileHandle, nIoResult, sFileName, nFileSize);

int x,y;
float th;

initConnections();
string mp="mapaA.txt";
int color = 0;

//int light = LSvalNorm(lightSensor);
//if(light < 18)  // If the Light Sensor reads a value less than 45:
//{
//  nxtDisplayTextLine(1, "Veo negro");                  // Motor C is run at a 20 power level.
//  mp = "mapaB.txt";
//  color = 1;
//}
//else                               // If the Light Sensor reads a value greater than or equal to 45:
//{
//  nxtDisplayTextLine(1, "Veo blanco");                 // Motor C is run at a 60 power level.
//  mp = "mapaA.txt";
//  color = 0;
//}

if(	loadMap(mp,connectionsMatrix[0][0]) ){
  nxtDisplayTextLine(6, "Mapa loaded ok");
}else{
  nxtDisplayTextLine(6, "Mapa NOT loaded");
}

/*drawMap();
th=-pi/4 + (20*PI)/180;
for (x=50; x<sizeX*sizeCell; x=x+200){
  th=th+PI/4;
  for (y=50; y<sizeY*sizeCell; y=y+200){
        drawRobot(x,y,th);
        wait1Msec(100);
  }
}*/
eraseDisplay();

// reset odometry values and motor encoders.
  nMotorEncoder[motorC] = 0;
  nMotorEncoder[motorA] = 0;
  robot_odometry.th = 0;
  robot_odometry.x = 0;
  robot_odometry.y = 0;

	StartTask(updateOdometry);
	if (color == 0) {
		planPath(PI,1,7,1,3);
		planPath((PI/2),1,3,3,3);
	} else {
		//planPath(PI,5,7,5,3);
	}


  float startX = 0;
  float startY = 0;
  float rotSpeed = 0;
  float heading = 0;



	//CAMERA TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	bool continueTracking = true;
  int _nblobs;
  blob_array _blobs;
  bool _condensed = true;

  // Initialise the camera
	NXTCAMinit(cam);
	bool initOdometry = false;
	HTGYROstartCal(HTGYRO);
	time1[T1] = 0;
	AcquireMutex(semaphore_odometry);
				  startX = robot_odometry.x;
				  startY = robot_odometry.y;
				  ReleaseMutex(semaphore_odometry);
	while (continueTracking) {

		// Get the blobs from the camera into the array
		 _nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
		 nxtDisplayTextLine(6, "%d", _nblobs);

		 // Select blob of COLOR to be tracked
		 int found = 0;

		 for (int i = 0; i < _nblobs; i++) {

     	if (_blobs[i].colour == RED /*&& _blobs[i].size > AREA_COLOR*/) {
     		found = 1;
     		if (!initOdometry) {
 					//TODO: remove this
				  initOdometry = true;
				}
     		nxtDisplayTextLine(3, "%d %d %d %d", _blobs[i].x1, _blobs[i].y1, _blobs[i].x2, _blobs[i].y2);
     		nxtDisplayTextLine(5, "%d", _blobs[i].size);
     		float w = alignToBall((_blobs[i].x1 + _blobs[i].x2)/2);
     		//Todo: if w is too big v = 0;
	     	float v = speedToBall(_blobs[i].size);
	     	setSpeed(v, w);
	     	if (_blobs[i].size > DESIRED_AREA) {
	     		nxtDisplayTextLine(3, "stop tracking");
	     		continueTracking = 0;
	     	}
     	}

     }

     if (!found) {
       //setSpeed(0, 0.5);
     nxtDisplayTextLine(3, "not found");
     wait1Msec(300);
     setSpeed(0, -0.5);



     }

     rotSpeed = HTGYROreadRot(HTGYRO);
			heading += rotSpeed * time1[T1] * 0.001;
		time1[T1] = 0;

		 // Give v and w to the robot according to distance and offset of the blob
		 // If goal reached: continueTracking =0


	}


	// Catch the ball?
	setSpeed(0.2, 0);
	wait1Msec(800);
	motor[motorB] = 15;
	setSpeed(0,0);
	wait1Msec(600);
  motor[motorB] = 0;

	//END CAMERA TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  AcquireMutex(semaphore_odometry);
  float endX = robot_odometry.x;
  float endY = robot_odometry.y;
  ReleaseMutex(semaphore_odometry);


	float diffX = endX - startX;
	float diffY = -1*(endY - startY);

	nxtDisplayTextLine(3, "x %f", diffY);
	nxtDisplayTextLine(4, "y %f",diffX);
	float columnX = diffX / 0.4;
	float columnY = diffY / 0.4;

	int newYcolumn = 0;
	if (columnY > 0) {
		newYcolumn = (int)(columnY+0.5);
	} else {
		newYcolumn = (int)(columnY-0.5);
	}
	int newXcolumn = (int)(columnX+0.5);
	newXcolumn =newXcolumn + 3;
	newYcolumn = newYcolumn + 3;
	nxtDisplayTextLine(5, "x %d y %d", newYcolumn, newXcolumn;
	nxtDisplayTextLine(6, "%f", heading);
	align2(heading);
	wait1Msec(5000);

	goToClosestExit(0,newYcolumn,newXcolumn,x_left,y_left,x_right,y_right);
	findExit(GREEN,BLUE);

	StopTask(updateOdometry);

  Close(hFileHandle, nIoResult);

  wait1Msec(60000);
}
