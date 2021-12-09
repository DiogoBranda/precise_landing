#ifndef UAVCONTROL_H
#define UAVCONTROL_H


#define LAND_HIGH  1
#define LAND_SPEED  -0.2
#define ERROR_TH  2
#define LAND_ERROR_TH  3
#define VEL_TH  0.3

#include <cmath>
#include <iostream>
class uavControl
{
public:
  uavControl();
  int p(float x, float y, float z, float *vx, float *vy, float *vz,float k);
};

#endif // UAVCONTROL_H
