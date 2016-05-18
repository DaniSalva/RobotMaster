// GLOBAL VARIABLE FOR ODOMETRY
#define INI_X 0
#define INI_Y 0
#define INI_TH 0

typedef struct {
     float x;   //x coordinate
     float y;   //y coordinate
     float th;  //theta - orientation RADIANES
}TPosition;


void set_position(TPosition &pos, float x, float y, float th)
{
  pos.x=x;
  pos.y=y;
  pos.th=th;

}

void normalizeAngle(float & angle){
// normalize angle in radians between PI and -PI

  while (angle>PI || angle<-PI){
    if (angle>PI){
      angle=angle-2*PI;
    }
    if (angle<-PI){
      angle=angle+2*PI;
    }
  }

}
