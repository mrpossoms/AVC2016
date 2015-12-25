#ifndef AVC_GPS
#define AVC_GPS

#include "types.h"

int     gpsHasNewReadings(int fd);
vec3f_t gpsGetReadings(int fd);

#endif