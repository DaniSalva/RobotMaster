
#include "positionLib.c"
#include "odometryAndMovement.c"
#include "drivers/mindsensors-nxtcam.h"

// variables globales de las dimensiones del mapa
int sizeX;
int sizeY;
int sizeCell;
int pixPerX;
int pixPerY;

// DEFINIR EL MAXIMO NUMERO DE CELDAS QUE PODEMOS UTILIZAR
#define MAX_X 7
#define MAX_Y 8
#define SONAR_THRES 40

#define RED 0
#define GREEN 2
#define BLUE 1

bool connectionsMatrix[2*MAX_X+1][2*MAX_Y+1];
int nf1Matrix[MAX_X][MAX_Y];
int pathX[MAX_X*MAX_Y];
int pathY[MAX_X*MAX_Y];


void initNF1(){
	for(int i=0; i<MAX_X; ++i){
		for (int j=0; j<MAX_Y; ++j){
			nf1Matrix[i][j]=-2;
		}
	}

}

void initConnections(){
	for(int i=0; i<2*MAX_X+1; ++i){
		for (int j=0; j<2*MAX_Y+1; ++j){
			connectionsMatrix[i][j]=false;
		}
	}

}

/* pasar de una celda(cellX,cellY) y un indice de vecindad numNeigh,
al correspondiente punto en la matriz de conexiones (connX,connY) */
void cell2connCoord(int cellX, int cellY, int numNeigh, int & connX, int & connY){
	connX=2*cellX+1;
	connY=2*cellY+1;
	switch(numNeigh){
	case 0: connY++; break;
	case 1: connY++; connX++; break;
	case 2: connX++;break;
	case 3: connY--; connX++; break;
	case 4: connY--; break;
	case 5: connY--; connX--; break;
	case 6: connX--; break;
	case 7: connY++; connX--; break;
	}
}

void cell2Coord(int cellX, int cellY, int numNeigh, int & connX, int & connY){
	connX=cellX;
	connY=cellY;
	switch(numNeigh){
	case 0: connY++; break;
	case 1: connY++; connX++; break;
	case 2: connX++;break;
	case 3: connY--; connX++; break;
	case 4: connY--; break;
	case 5: connY--; connX--; break;
	case 6: connX--; break;
	case 7: connY++; connX--; break;
	}
}

void setConnection(int cellX, int cellY, int numNeigh){
	int connX, connY; // coordinates in the connection matrix
	// from coordinates in the grid of cells to coordinates in the connection matrix
	cell2connCoord(cellX, cellY, numNeigh, connX, connY);
	connectionsMatrix[connX][connY]=true;
}

void deleteConnection(int cellX, int cellY, int numNeigh){
	int connX, connY; // coordinates in the connection matrix
	// from coordinates in the grid of cells to coordinates in the connection matrix
	cell2connCoord(cellX, cellY, numNeigh, connX, connY);
	connectionsMatrix[connX][connY]=false;
}

bool isConnected(int cellX, int cellY, int numNeigh){
	int connX, connY; // coordinates in the connection matrix
	// from coordinates in the grid of cells to coordinates in the connection matrix
	cell2connCoord(cellX, cellY, numNeigh, connX, connY);
	return(connectionsMatrix[connX][connY]);

}


bool readLineHeader(TFileHandle hFileHandle,TFileIOResult nIoResult, int & dimX, int & dimY, int &dimCell)
{
	//unsigned ans;
	//short ans;
	int ind = 1;
	//float aux = 0.1;
	bool eol = false;
	bool endfile = false;
	char onechar;
	char linechar[40];
	int num = 0;
	int indNum=0;
	int numbersRead[3];

	// read header
	while(!eol){
		ReadByte(hFileHandle, nIoResult, onechar);
		if ( nIoResult == 0 ){ // all ok
			if (onechar==13) // end of line
			{
				linechar[ind-1]=0;
				eol=true;
			}
			else{
				if (onechar=='\n'){ // line jump
					//skip
				}
				else{
					linechar[ind-1]=onechar;
					if (onechar==' '){
						numbersRead[indNum]=num;
						num=0;
						indNum++;
						}else{
						num = 10*num + (onechar - '0' );
					}
					ind++;
				}
			}
		}
		else{
			if (nIoResult==ioRsltEndOfFile){
				eol=true;
				endfile=true;

				}else{
				nxtDisplayTextLine(1, "PROBLEM READING map");
			}
		}
	}
	// from char to string
	//StringFromChars(linestring,linechar);
	if (numbersRead[indNum]!=num && num!=0){
		numbersRead[indNum]=num;
	}

	dimX = numbersRead[0];
	dimY = numbersRead[1];
	dimCell = numbersRead[2];

	/*nxtDisplayTextLine(3, "%d %d ", dimX, dimY);
	nxtDisplayTextLine(4, "%d ", dimCell);
	wait10Msec(300);*/

	return endfile;
}





bool readNextLine(TFileHandle hFileHandle,TFileIOResult & nIoResult, int & mapRow)
{
	//short ans;

	int ind = 0; // pointer to keep all text read in vector linechar
	char linechar[(MAX_X*2+1)*3]; // how long do we expect the lines...
	string linestring;
	char onechar;

	bool eol = false;
	bool endfile = false;
	int mapCol=0;

	// read header
	while(!eol){
		ReadByte(hFileHandle, nIoResult, onechar);
		if ( nIoResult == 0 ){ // all ok
			if (onechar==13) // end of line
			{
				linechar[ind]=0;
				eol=true;
			}
			else{
				if (onechar=='\n'){ // line jump
					//skip
				}
				else{
					linechar[ind]=onechar;
					if (onechar==' '){
						//numbersRead[indNum]=num;
						//num=0;
						//indNum++;
						}else{
						if (onechar=='1'){
							nxtDisplayTextLine(3, " %d %d", mapCol,mapRow);
							connectionsMatrix[mapCol][mapRow]=true;
						}
						// else { false} // by default is false
						mapCol++;

					}
					ind++;
				}
			}
		}
		else{
			if (nIoResult==ioRsltEndOfFile){
				eol=true;
				endfile=true;

				}else{
				nxtDisplayTextLine(1, "PROBLEM READING map");
			}
		}
	}

	// jump to next row
	mapRow--;
	if (mapRow<0){
		// STOP READING, map is full
		endfile=true;
	}

	// from char to string
	StringFromChars(linestring,linechar);
	/*if (numbersRead[indNum]!=num && num!=0){
	numbersRead[indNum]=num;
	}*/

	nxtDisplayTextLine(3, "%s ", linestring);

	/*for(int j=2; j<=indNum; ++j){
	setConnection(numbersRead[0], numbersRead[1], numbersRead[j]);
	nxtDisplayTextLine(4, "%d connection open", numbersRead[j]);
	//wait10Msec(200);
	}*/
	return endfile;
}

////////////////////////////////////////////////////////////////
// load map from a txt to the occupancy and connection matrix
// FILL GLOBAL VARIABLES dimX dimY cellSize
bool loadMap(string mapFileName, bool &connections)
{
	//byte handle;
	//int um,x,y;
	//string buf,msg;
	bool loadingOk=false;
	int dimConectionX,dimConectionY;
	int mapRow; // last row from connection matrix?

	string line="";
	bool eof = false;
	TFileIOResult nIoResult;
	TFileHandle hFileHandle;
	int nFileSize = 0; // it is filled when we open file

	CloseAllHandles(nIoResult);
	hFileHandle = 0;
	//nxtDrawLine(_x+2, _y, _x-2, _y);

	OpenRead(hFileHandle, nIoResult, mapFileName, nFileSize);
	if( nIoResult ==0 ){
		nxtDisplayTextLine(1, "OPEN OK: %d", nFileSize);

		//StringFromChars(sToString, FromChars)
		//Converts an array of bytes to a string value.  You MUST end your char array with a char value of zero!

		// read first line
		eof = readLineHeader(hFileHandle,nIoResult, sizeX, sizeY, sizeCell);
		//nxtDisplayTextLine(2, "%s", line);
		mapRow=2*sizeY;
		// read rest of data
		while(!eof){
			eof = readNextLine(hFileHandle,nIoResult, mapRow);
			//eof = readNextCellConnections(hFileHandle,nIoResult);
			//nxtDisplayTextLine(2, "%s", line);
		}
		loadingOk=true;
		Close(hFileHandle, nIoResult);
	}
	else{
		loadingOk=false;
		nxtDisplayTextLine(1, "PROBLEM OPENING file");
	}

	return loadingOk;
}


// DRAW map and robot


void drawMap(){
	int i,j,cx,cy;

	eraseDisplay(); // L_B: (0,0); T_T: (99,63)
	//nxtDrawRect(_l, _t, _r, _b);
	pixPerX=100/sizeX;
	pixPerY=64/sizeY;

	nxtDrawRect(0,sizeY*pixPerY,sizeX*pixPerX,0);

	//nxtDrawLine(xPos, yPos, xPosTo, yPosTo);
	//i=cellX*sizeY+cellY;

	// check "vertical" walls
	for (i=2; i<2*sizeX; i=i+2){
		for (j=1; j< 2*sizeY; j=j+2){
			if (connectionsMatrix[i][j]==false){
				// paint "right" wall from cell (i/2-1, j2-1)
				cx=(i-1)/2;
				cy=(j-1)/2;
				nxtDrawLine((cx+1)*pixPerX, cy*pixPerY, (cx+1)*pixPerX, (cy+1)*pixPerY);
			}
		}
	}


	// check "horizontal" walls
	for (j=2; j<2*sizeY; j=j+2){
		for (i=1; i< 2*sizeX; i=i+2){
			if (connectionsMatrix[i][j]==false){
				// paint "top" wall from cell (i-1)/2, (j-1)/2)
				cx=(i-1)/2;
				cy=(j-1)/2;
				nxtDrawLine((cx)*pixPerX, (cy+1)*pixPerY, (cx+1)*pixPerX, (cy+1)*pixPerY);
			}
		}
	}

}

/* Convert from robot odometry coordinates (in mm) to cell coordinates */
void pos2cell(float x_mm, float y_mm, int & x_cell, int & y_cell){

	x_cell =  (int) x_mm/sizeCell;

	y_cell = (int) y_mm/sizeCell;

}

void drawRobot(float x_mm, float y_mm, float ang_rad){
	int cellx,celly;
	int pixX,pixY;
	float ang_grad;
	int th;

	pos2cell(x_mm, y_mm, cellx,celly);

	pixX=cellx*pixPerX+pixPerX/2;
	pixY=celly*pixPerY+pixPerY/2;
	nxtFillEllipse(pixX-1, pixY+1, pixX+1, pixY-1); //nxtFillEllipse(Left, Top, Right, Bottom);

	//normalizeAngle(ang_rad);
	ang_grad=radiansToDegrees(ang_rad);
	if (ang_grad<0){ ang_grad=ang_grad+360;}
	th=(ang_grad+22.5)/45;
	while(th>7){th=th-8;}

	//paint orientation
	if(th==0)		    { nxtDrawLine(pixX,pixY,pixX+2,pixY);		}
	else if(th==1)	{ nxtDrawLine(pixX,pixY,pixX+2,pixY+2);	}
	else if(th==2)	{ nxtDrawLine(pixX,pixY,pixX,pixY+2);	  }
	else if(th==3)	{ nxtDrawLine(pixX,pixY,pixX-2,pixY+2);	}
	else if(th==4)	{ nxtDrawLine(pixX,pixY,pixX-2,pixY);		}
	else if(th==5)	{ nxtDrawLine(pixX,pixY,pixX-2,pixY-2);	}
	else if(th==6)	{ nxtDrawLine(pixX,pixY,pixX,pixY-2);		}
	else if(th==7)	{ nxtDrawLine(pixX,pixY,pixX+2,pixY-2);	}

}

void fillNF1Matrix(int x, int y, int value) {

	int elements = 1;
	int waveX[120];
	int waveY[120];
	waveX[0] = x;
	waveY[0] = y;
	int newWaveX[120];
	int newWaveY[120];
	int newElements = 0;
	int connX, connY;
	while(elements != 0 && newElements<120) {
		for(int i=0; i < elements; i++) {
			for(int j = 0; j < 8; j = j + 2) {
				if (isConnected(waveX[i],waveY[i], j)) {
					cell2Coord(waveX[i], waveY[i], j, connX, connY);
					if (nf1Matrix[connX][connY] == -2) {
						newWaveX[newElements] = connX;
						newWaveY[newElements] = connY;
						newElements++;
						if(newElements==120){
							break;
						}
					}
				}
			}
			if(newElements==120){
				break;
			}
		}
		//Actualizar los expandidos
		for(int i=0; i < elements; i++) {
			nf1Matrix[waveX[i]][waveY[i]] = value;
		}
		value = value + 1;
		for(int i=0; i < newElements; i++) {
			waveX[i]= newWaveX[i];
			waveY[i]= newWaveY[i];
		}
		elements = newElements;
		newElements = 0;
	}
}



int rotMovement(float previousDir, float goalth){
	int move=0;
	float result= previousDir-goalth;
	if(result==0){
		move=0;
	}
	else if(result>(-1.6) && result<(-1.5)){
		move=-1;
		}else if(result<(1.6) && result>(1.5)){
		move=1;
		}else if(result>(-4.8) && result<(-4.7)){
		move=1;
		}else if(result<(4.8) && result>(4.7)){
		move=-1;
		}else if(result<(3.2) && result>(3.1)){
		move=2;
		}else if(result>(-3.2) && result<(-3.1)){
		move=-2;
	}
	return move;
}

// go from CURRENT position (odometry) to (middle??) of cell (cellX, cellY)
bool go(int cellX, int cellY, float previousDir, float th, int x, int y, int x_end, int y_end, bool replan){

	int move =rotMovement(previousDir,th);
	align(move);
	if(SensorValue(SONAR) < SONAR_THRES){
		return false;
	}
	else{
		fordwardSonar(0.4,true);
		return true;
	}

}

float planPath(float previousDir,int x_ini, int y_ini, int x_end, int y_end, bool replan){
	// Store in pathX and pathY respectively the coordinates of all the cells that the robot has to cross to reach the goal.
	int x = x_ini;
	int y = y_ini;
	int nextNeighbourX = 0;
	int nextNeighbourY = 0;
	int connX, connY;
	int direction = 0;
	float th = 0;

	// NF1
	initNF1();
	fillNF1Matrix(x_end, y_end, 0);

	int heuristica = nf1Matrix[x_ini][y_ini];

	while(heuristica != 0) {
		for(int i = 0; i < 8; i = i + 2) {
			if (isConnected(x,y, i)) {
				cell2Coord(x, y, i, connX, connY);
				if(nf1Matrix[connX][connY] < heuristica) {
					heuristica = nf1Matrix[connX][connY];
					nextNeighbourX = connX;
					nextNeighbourY = connY;
					direction = i;
				}
			}
		}

		switch(direction){
		case 0: th = 0; break;
		case 2: th = -(PI/2);break;
		case 4: th = PI; break;
		case 6: th = PI/2; break;
		}
		bool update = go(nextNeighbourX, nextNeighbourY, previousDir, th, x, y, x_end, y_end,replan);
		previousDir=th;

		if (update) {
			x = nextNeighbourX;
			y = nextNeighbourY;
		}
		else{	//RePlan path
			deleteConnection(x, y, direction);
			initNF1();
			fillNF1Matrix(x_end, y_end, 0);
			drawMap();
			heuristica = nf1Matrix[x_ini][y_ini];
		}
	}
	return th;
}

float euclidianDistance(int x_1,int y_1, int x_2, int y_2){

	return sqrt(pow((x_2-x_1),2)+pow((y_2-y_1),2));
}

void goToClosestExit(float thinit,int xpos, int ypos,int xleft, int yleft,int xright, int yright){
	float th=0;
	if(euclidianDistance(xpos,ypos,xleft,yleft)< euclidianDistance(xpos,ypos,xright,yright)){
		th=planPath(thinit,xpos,ypos,xleft,yleft,false);
	}
	else{
		th=planPath(thinit,xpos,ypos,xright,yright,false);
	}
	int move=rotMovement(th,0);
	align(move);
}

void goRightExit(int x, int y,int exit_xr,int exit_yr){
	float th=planPath(0,x,y,exit_xr,exit_yr,false);
	int move=rotMovement(th,0);
	align(move);
	fordward(0.4);
}

void goLeftExit(int x, int y,int exit_xl,int exit_yl){
	float th=planPath(0,x,y,exit_xl,exit_yl,false);
	int move=rotMovement(th,0);
	align(move);
	fordward(0.4);
}

//side: 0 izquierda, 1 derecha
void findExit(int side){
	int exit_x_left=0;
	int exit_y_left=0;
	int exit_x_right=0;
	int exit_y_right=0;

	int start_x_left=0;
	int start_y_left=0;
	int start_x_right=0;
	int start_y_right=0;

	int goalColor;
	int otherColor;

	if(side==0){
		exit_x_left=3;
		exit_y_left=7;
		exit_x_right=6;
		exit_y_right=7;

		start_x_left=4;
		start_y_left=6;
		start_x_right=5;
		start_y_right=6;
		goalColor=BLUE;
		otherColor=GREEN;
	}
	else{
		exit_x_left=0;
		exit_y_left=7;
		exit_x_right=3;
		exit_y_right=7;

		start_x_left=1;
		start_y_left=6;
		start_x_right=2;
		start_y_right=6;
		goalColor=GREEN;
		otherColor=BLUE;
	}
	int _nblobs;
	bool foundGoal=false;
	double centerGoal=0;
	double centerOther=0;
	bool foundOther=false;
	blob_array _blobs;
	bool _condensed = true;
	// Initialise the camera
	NXTCAMinit(cam);
	bool initOdometry = false;

	_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
	centerGoal=0;
	centerOther=0;
	foundGoal=false;
	foundOther=false;
	for (int i = 0; i < _nblobs; i++) {
		if ((_blobs[i].colour ==  goalColor) && (_blobs[i].size>200)){
			foundGoal = true;
			double centerGoal=(_blobs[i].x1 + _blobs[i].x2)/2;
		}
		else if ((_blobs[i].colour ==  otherColor) && (_blobs[i].size>200)){
			foundOther=true;
			double centerOther=(_blobs[i].x1 + _blobs[i].x2)/2;
		}
	}

	if(!(foundGoal || foundOther)){
		nxtDisplayTextLine(1, "NINGUNO");
		while(!foundGoal && !foundOther){
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			centerGoal=0;
			centerOther=0;
			foundGoal=false;
			foundOther=false;
			for (int i = 0; i < _nblobs; i++) {
				if (_blobs[i].colour ==  goalColor){
					foundGoal = true;
					double centerGoal=(_blobs[i].x1 + _blobs[i].x2)/2;
				}
				else if (_blobs[i].colour ==  otherColor){
					foundOther=true;
					double centerOther=(_blobs[i].x1 + _blobs[i].x2)/2;
				}
			}
			if (!foundGoal && !foundOther) {


				wait1Msec(300);
				if (side == 0) {
					setSpeed(0, -0.5);
					} else {
					setSpeed(0, 0.5);
				}
			}
		}
	}
	nxtDisplayTextLine(2, "goal %f",centerGoal);
	nxtDisplayTextLine(3, "other %f",centerOther);
	setSpeed(0, 0);
	if(foundGoal && foundOther){
		if(centerGoal<centerOther){
			if(side==0){
				nxtDisplayTextLine(1, "2 izq-dere");
				goRightExit(start_x_left,start_y_left,exit_x_right,exit_y_right);
			}
			else{
				nxtDisplayTextLine(1, "2 dere-dere");
				goRightExit(start_x_right,start_y_right,exit_x_right,exit_y_right);
			}

		}
		else{
			if(side==0){
				nxtDisplayTextLine(1, "2 izq-izq");
				goLeftExit(start_x_left,start_y_left,exit_x_left,exit_y_left);
			}
			else{
				nxtDisplayTextLine(1, "2 dere-izq");
				goLeftExit(start_x_right,start_y_right,exit_x_left,exit_y_left);
			}
		}
	}
	else if(foundGoal){
		if(side==0){
			nxtDisplayTextLine(1, "1 izq-izq");
			goLeftExit(start_x_left,start_y_left,exit_x_left,exit_y_left);
		}
		else{
			nxtDisplayTextLine(1, "1 dere-dere");
			goRightExit(start_x_right,start_y_right,exit_x_right,exit_y_right);
		}
	}
	else if(foundOther){
		if(side==0){
			nxtDisplayTextLine(1, "1 izq-dere");
			goRightExit(start_x_left,start_y_left,exit_x_right,exit_y_right);
		}
		else{
			nxtDisplayTextLine(1, "1 dere-izq");
			goLeftExit(start_x_right,start_y_right,exit_x_left,exit_y_left);
		}
	}
}
