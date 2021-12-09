#include "uavcontrol.h"

uavControl::uavControl()
{

}

int uavControl::p(float x, float y, float z, float *vx, float *vy, float *vz,float k)
{
  if(z<-0.1){
    return -1;
  }
  else {
    if(z > LAND_HIGH){
      if(z < LAND_ERROR_TH){
        if( abs(x) > ERROR_TH || abs(x) > ERROR_TH )
        {
          *vx = k*x;
          *vy = k*y;
          *vz = 0;
        }
        else {
          *vx = k*x;
          *vy = k*y;
          *vz = LAND_SPEED;
        }
      }
      else {
        *vx = k*x;
        *vy = k*y;
        *vz = LAND_SPEED;
      }

    }
    else {
      *vx = 0;
      *vy = 0;
      *vz = 0;
      return 2;
    }
    if(abs(*vx)>VEL_TH)
      *vx=VEL_TH;
    if(abs(*vy)>VEL_TH)
      *vy=VEL_TH;

  }
  return 1;
}
