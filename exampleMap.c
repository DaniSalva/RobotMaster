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

/********************************************************/
/*************STEP: SELECT COLOR AND MAP*****************/
/********************************************************/
int light = LSvalNorm(lightSensor);
light = 30; //REMOVE THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
if(light < 18)  // If the Light Sensor reads a value less than 45:
{
  //nxtDisplayTextLine(1, "Veo negro");                  // Motor C is run at a 20 power level.
  mp = "mapaB.txt";
  color = 1;
}
else                               // If the Light Sensor reads a value greater than or equal to 45:
{
  //nxtDisplayTextLine(1, "Veo blanco");                 // Motor C is run at a 60 power level.
  mp = "mapaA.txt";
  color = 0;
}

if(	loadMap(mp,connectionsMatrix[0][0]) ){
  //nxtDisplayTextLine(6, "Mapa loaded ok");
}else{
  //nxtDisplayTextLine(6, "Mapa NOT loaded");
}

drawMap();
/*th=-pi/4 + (20*PI)/180;
for (x=50; x<sizeX*sizeCell; x=x+400){
  th=th+PI/4;
  for (y=50; y<sizeY*sizeCell; y=y+400){
        drawRobot(x,y,th);
        wait1Msec(100);
  }
}*/
//eraseDisplay();

// reset odometry values and motor encoders.
  nMotorEncoder[motorC] = 0;
  nMotorEncoder[motorA] = 0;
  robot_odometry.th = 0;
  robot_odometry.x = 0;
  robot_odometry.y = 0;

  /********************************************************/
	/*************ZIG ZAG AND GO TO BALL ROOM****************/
	/********************************************************/
	StartTask(updateOdometry);
	if (color == 0) {
		//planPath(PI,1,7,1,3);
		//planPath((PI/2),1,3,3,3);
	} else {
		//planPath(PI,5,7,5,3);
		planPath(-(PI/2),5,3,3,3);
	}

  /********************************************************/
	/*******************FIND BALL****************************/
	/********************************************************/

	float rotSpeed = 0;
  float heading = 0;

	bool continueTracking = true;
  int _nblobs;
  blob_array _blobs;
  bool _condensed = true;

  // Initialise the camera
	NXTCAMinit(cam);
	HTGYROstartCal(HTGYRO);
	time1[T1] = 0;
	AcquireMutex(semaphore_odometry);
	float aux_odometryX = robot_odometry.x;
	float	aux_odometryY = robot_odometry.y;
	float	aux_odometryTH = robot_odometry.th;
	robot_odometry.x = 0;
	if (color == 0) {
		robot_odometry.y = 0;
	} else {
		robot_odometry.y = 1.2;
	}
	robot_odometry.th = 0;
	ReleaseMutex(semaphore_odometry);
	if (color == 1) {
		aux_odometryY = aux_odometryY - 1.2;
	}
	while (continueTracking) {

		// Get the blobs from the camera into the array
		 _nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
		 //nxtDisplayTextLine(6, "%d", _nblobs);

		 // Select blob of COLOR to be tracked
		 int found = 0;

		 for (int i = 0; i < _nblobs; i++) {

     	if (_blobs[i].colour == RED /*&& _blobs[i].size > AREA_COLOR*/) {
     		found = 1;
     		//nxtDisplayTextLine(3, "%d %d %d %d", _blobs[i].x1, _blobs[i].y1, _blobs[i].x2, _blobs[i].y2);
     		//nxtDisplayTextLine(5, "%d", _blobs[i].size);
     		float w = alignToBall((_blobs[i].x1 + _blobs[i].x2)/2);
     		//Todo: if w is too big v = 0;
	     	float v = speedToBall(_blobs[i].size);
	     	setSpeed(v, w);
	     	if (_blobs[i].size > DESIRED_AREA) {
	     		//nxtDisplayTextLine(3, "stop tracking");
	     		continueTracking = 0;
	     	}
     	}

     }

     if (!found) {
				//nxtDisplayTextLine(3, "not found");
				wait1Msec(300);
				if (color == 0) {
					setSpeed(0, -0.5);
				} else {
					setSpeed(0, 0.5);
				}



     }

     rotSpeed = HTGYROreadRot(HTGYRO);
			heading += rotSpeed * time1[T1] * 0.001;
		time1[T1] = 0;

		 // Give v and w to the robot according to distance and offset of the blob
		 // If goal reached: continueTracking =0


	}


	/********************************************************/
	/****************CATCH THE BALL**************************/
	/********************************************************/
	setSpeed(0.2, 0);
	wait1Msec(800);
	motor[motorB] = 15;
	setSpeed(0,0);
	wait1Msec(590);
  motor[motorB] = 0;

	/********************************************************/
	/************GO TO PROPER POSITION TO EXIT***************/
	/********************************************************/
	align2(heading);



  AcquireMutex(semaphore_odometry);
  float endX = robot_odometry.x;
  float endY = robot_odometry.y;
  robot_odometry.th = 0;
  ReleaseMutex(semaphore_odometry);
  float foundBallAuxX = endX;
  float foundBallAuxY = endY;

  float columnX = foundBallAuxX / 0.4;
 	float columnY = foundBallAuxY / 0.4;

 	int newYcolumn = 0;
 	if (columnY > 0) {
 		newYcolumn = (int)(columnY+0.5);
 	} else {
 		newYcolumn = (int)(columnY-0.5);
 	}
 	int newXcolumn = (int)(columnX+0.5);
 	int foundBallX =newYcolumn + 3;
 	int foundBallY = newXcolumn + 3;
 	//nxtDisplayTextLine(6, "%f", heading);
 	drawFindBall(3,3,foundBallX, foundBallY);
 	drawRobot(foundBallX*400,foundBallY*400,0);
 	drawRobot(foundBallX*400,foundBallY*400,PI);
 	drawRobot(foundBallX*400,foundBallY*400,-(PI/2));
 	drawRobot(foundBallX*400,foundBallY*400,PI/2);
 	if (color == 0) {
 		drawFindBall(foundBallX, foundBallY, 4, 6);
	} else {
		drawFindBall(foundBallX, foundBallY, 2, 6);
	}

  float goalY = 1.2;
  float goalX = 0.4;
  if (color == 1) {
  	goalX = 0.8;
  }

  //nxtDisplayTextLine(3, "x %f", endY);
	//nxtDisplayTextLine(4, "y %f",endX);
	while (endX < goalY) {
		setSpeed(0.15,0);
		wait1Msec(100);
		AcquireMutex(semaphore_odometry);
	  endX = robot_odometry.x;
	  ReleaseMutex(semaphore_odometry);
	}
	setSpeed(0,0);
	if(endY > goalX) {
		align2(90);
		while (endY > goalX) {
			setSpeed(0.15,0);
			wait1Msec(100);
			AcquireMutex(semaphore_odometry);
		  endY = robot_odometry.y;
		  ReleaseMutex(semaphore_odometry);
		}
		setSpeed(0,0);
		align2(-90);
	} else {
		align2(-90);
		while (endY < goalX) {
			setSpeed(0.15,0);
			wait1Msec(100);
			AcquireMutex(semaphore_odometry);
		  endY = robot_odometry.y;
		  ReleaseMutex(semaphore_odometry);
		}
		setSpeed(0,0);
		align2(90);
	}

	//Reset proper odometry
	AcquireMutex(semaphore_odometry);
  robot_odometry.x = robot_odometry.x + aux_odometryX;
	robot_odometry.y = robot_odometry.y + aux_odometryY;
	robot_odometry.th = robot_odometry.th + aux_odometryTH;
  ReleaseMutex(semaphore_odometry);

  /********************************************************/
	/********************EXIT MAZE*************************/
	/******************************************************/


	//TODO: this for different maps
	findExit(GREEN,BLUE, 0);

	StopTask(updateOdometry);

  Close(hFileHandle, nIoResult);

  wait1Msec(60000);
}
