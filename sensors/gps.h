#ifndef AVC_GPS
#define AVC_GPS

#include "types.h"

int gpsInit(const char* device);
int gpsHasNewReadings();
int gpsGetReadings(vec3f_t* position, vec3f_t* veclocity);

#endif