#include "mutexLib.c"
#include "positionLib.c"
#include "drivers/hitechnic-gyro.h"
#include "drivers/common.h"
// ROBOT PARAMETERS
float R=0.028; // m
float L=0.11; // m

#define SONAR_T 30


int gyroR=94;
int gyroL=86;

TPosition robot_odometry;       // WE SHOULD ACCESS THIS VARIABLE with a "semaphore".
TMutex semaphore_odometry = 0;  // Important to initialize to zero!!! Not acquired.

//File
const string sFileName = "prueba.txt";

TFileIOResult nIoResult;
TFileHandle hFileHandle;
int nFileSize           = 9000;


// FUNCTION!!
int setSpeed(float v, float w)
{

// start the motors so that the robot gets v m/s linear speed and w RADIAN/s angular speed

  float w_l =1, w_r=1;	//velocidad angular de cada rueda
  float m = 6.095277901134592;
  float n = 0.15899967782102523;
  float motorPowerRight, motorPowerLeft;	//power al motor izquierdo y derecho

  w_r = ((1/R) * v) + L/(2*R) * w;
  w_l = 1/R * v - L/(2*R) * w;

  motorPowerRight = m * w_r - n;
  motorPowerLeft = m * w_l - n;
  //nxtDisplayTextLine(6, "%f", motorPowerLeft);
  //nxtDisplayTextLine(7, "%f", motorPowerRight);

	//System critic section
  hogCPU();
  motor[motorA] = motorPowerRight;
  motor[motorC] = motorPowerLeft;
  releaseCPU();

  return 1;
}

void align(int angle) {
	//float th = 0;
	//AcquireMutex(semaphore_odometry);
	//th = robot_odometry.th;
	//ReleaseMutex(semaphore_odometry);
	//if(angle == PI && th < 0) {
	//	angle = -PI;
	//}
	//int move=0;

	//float error = 0.2;
	//float minusError = angle-error;
	//normalizeAngle(minusError);
	//float plusError = angle+error;
	//normalizeAngle(plusError);

	//if(angle == PI || angle == -PI) {
	//	minusError = 3;
	//	plusError = -3;
	//	if(th<minusError && th>plusError){
	//		move=1;
	//	}
	//} else {
	//	if (th<minusError || th>plusError) {
	//		move=1;
	//	}
	//}
	if(angle!=0){ //0=not change direction
				//Init Gyro
		float rotSpeed = 0;
	  float heading = 0;
	  time1[T1] = 0;
	  int gyroRot=90;

	  HTGYROstartCal(HTGYRO);

		if(angle <0) { //Left
			setSpeed(0, 0.6);
			gyroRot=gyroL*angle; //rot 90 or 180
		} else {					//Right
			setSpeed(0, -0.6);
			gyroRot=gyroR*angle;
		}

		while (abs(heading)<abs(gyroRot))
	  {
	    // Wait until 20ms has passed
	    while (time1[T1] < 20)
	      wait1Msec(1);

	    // Reset the timer
	    time1[T1]=0;


	    rotSpeed = HTGYROreadRot(HTGYRO);
	    heading += rotSpeed * 0.02;
		}


		setSpeed(0,0);
	}

}

void align2(int angle) {
	if(angle!=0){ //0=not change direction
				//Init Gyro
		float rotSpeed = 0;
	  float heading = 0;
	  time1[T1] = 0;
	  int gyroRot=angle;

	  HTGYROstartCal(HTGYRO);



		if(angle <0) { //Left
			setSpeed(0, -0.6);
		} else {					//Right
			setSpeed(0, 0.6);
		}

		while (abs(heading)<abs(gyroRot))
	  {
	    // Wait until 20ms has passed
	    while (time1[T1] < 20)
	      wait1Msec(1);

	    // Reset the timer
	    time1[T1]=0;


	    rotSpeed = HTGYROreadRot(HTGYRO);
	    heading += rotSpeed * 0.02;
		}


		setSpeed(0,0);
	}

}

void fordward(float distance) {
	setSpeed(0.1,0);
	wait1Msec(4000);
	setSpeed(0,0);
}

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

// TASK TO BE LAUNCHED SIMULTANEOUSLY to "main"!!
task updateOdometry(){
  float cycle = 0.2 ; // we want to update odometry every ?? s
  float dS,dx,dy, dT;
  string sString;

  while (true){

   short timeAux=nPgmTime;
	// read tachometers, and estimate how many m. each wheel has moved since last update
    // RESET tachometer right after to start including the "moved" degrees turned in next iteration

   //System critic section 1
  	hogCPU();
   	float degL = nMotorEncoder[motorC];
   	float degR = nMotorEncoder[motorA];
   	nMotorEncoder[motorC] = 0;
   	nMotorEncoder[motorA] = 0;
   	releaseCPU();

   	float radL = degL * PI/180;
   	float radR = degR * PI/180;

   	float w_r = radR/cycle;
   	float w_l = radL/cycle;

   	float v = (R / 2) * w_r + (R / 2) * w_l;
   	float w = (R / L) * w_r + (-R / L) * w_l;

   	//nxtDisplayTextLine(1, "%f" "%f" "%f", nPgmTime, radL, PI/180);
   	dT = w*cycle;
   	dS = v * cycle;


   	float dx = dS * cos(robot_odometry.th + dT/2);
   	float dy = dS * sin(robot_odometry.th + dT/2);

   	AcquireMutex(semaphore_odometry);
		robot_odometry.th = robot_odometry.th + dT;
		normalizeAngle(robot_odometry.th);
   	robot_odometry.x = robot_odometry.x + dx;
   	robot_odometry.y = robot_odometry.y - dy;
   	ReleaseMutex(semaphore_odometry);

   	// show each step on screen and write in a file
		//nxtDisplayTextLine(2, "ODOMETRY NEW VALUE");
    	//nxtDisplayTextLine(3, "x,y: %2.2f %2.2f", robot_odometry.x,robot_odometry.y);
    	//nxtDisplayTextLine(4, "theta: %2.2f ", robot_odometry.th);
	  	// file ...
      //StringFormat(sString, "x: %2.2f , y: %2.2f , theta: %2.2f \n",
      //  robot_odometry.x, robot_odometry.y, robot_odometry.th);
        StringFormat(sString, "%2.2f ",
        robot_odometry.x);
      WriteText(hFileHandle, nIoResult, sString);
      StringFormat(sString, "%2.2f ",
        robot_odometry.y);
      WriteText(hFileHandle, nIoResult, sString);
      StringFormat(sString, "%2.2f\n",
        robot_odometry.th);
      WriteText(hFileHandle, nIoResult, sString);




	 // Wait until cycle is completed?
	 // ...
    	wait1Msec(cycle * 1000);

  }

}
