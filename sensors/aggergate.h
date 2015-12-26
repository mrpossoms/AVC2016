#ifndef AVC_SENSOR_AGGERGATE
#define AVC_SENSOR_AGGERGATE

#include "imu.h"
#include "gps.h"

#include "system.h"

int senInit(const char* imuDevice, const char* gpsDevice);
int senUpdate(fusedObjState_t* body);

#endif
