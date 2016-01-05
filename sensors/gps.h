#ifndef AVC_GPS
#define AVC_GPS

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

int gpsInit(const char* device);
int gpsHasNewReadings();
int gpsGetReadings(vec3f_t* position, vec3f_t* veclocity);

int gpsRouteLoad(const char* path, gpsWaypointCont_t** waypoints);
int gpsRouteAdvance(vec3f_t* position, gpsWaypointCont_t** current, uint8_t lapFlag);

#ifdef __cplusplus
}
#endif

#endif