#ifndef AVC_SENSOR_AGERGATE
#define AVC_SENSOR_AGERGATE

#include "imu.h"
#include "gps.h"

#include "system.h"

int senInit();
int senUpdate(fusedObjState_t* body);

#endif
