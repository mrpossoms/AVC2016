#ifndef AVC_SENSOR_AGGERGATE
#define AVC_SENSOR_AGGERGATE

#include "imu.h"
#include "gps.h"
#include "base/system.h"

#ifdef __cplusplus
extern "C" {
#endif

int senInit(const char* imuDevice, const char* gpsDevice, const char* calProfile);
int senUpdate(sensors_t* sensors);
int senShutdown();

#ifdef __cplusplus
}
#endif
#endif
