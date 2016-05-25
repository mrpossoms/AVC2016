#ifndef AVC_GPS
#define AVC_GPS

#include <libNEMA.h>

#include "base/types.h"

#define GPS_FIX 1

#ifdef __cplusplus
extern "C" {
#endif

extern gpsState_t GPS_STATE;

int gpsInit(const char* device);
int gpsShutdown();
int gpsHasNewReadings();
int gpsGetReadings(vec3f_t* position, vec3f_t* heading);

float gpsDistToWaypoint(vec3f_t* position, gpsWaypointCont_t* waypoint);
float gpsDistToWaypoint3D(vec3f_t* position, gpsWaypointCont_t* waypoint);
vec3f_t gpsHeadingToWaypoint(vec3f_t* position, gpsWaypointCont_t* waypoint);

int gpsRouteLoad(const char* path, gpsWaypointCont_t** waypoints);
int gpsRouteUnload(gpsWaypointCont_t** waypoints);
int gpsRouteAdvance(vec3f_t* position, gpsWaypointCont_t** current, uint8_t lapFlag);

void latLon2meters(vec3f_t* coord);

#ifdef __cplusplus
}
#endif

#endif
