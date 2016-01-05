#ifndef AVC_TYPES
#define AVC_TYPES

#include <inttypes.h>

#define vec3Add(r, v1, v2){\
	(r).v[0] = (v1).v[0] + (v2).v[0];\
	(r).v[1] = (v1).v[1] + (v2).v[1];\
	(r).v[2] = (v1).v[2] + (v2).v[2];\
}\

#define vec3Sub(r, v1, v2){\
	(r).v[0] = (v1).v[0] - (v2).v[0];\
	(r).v[1] = (v1).v[1] - (v2).v[1];\
	(r).v[2] = (v1).v[2] - (v2).v[2];\
}\

#define vec3Scl(r, v1, s){\
	(r).v[0] = (v1).v[0] * s;\
	(r).v[1] = (v1).v[1] * s;\
	(r).v[2] = (v1).v[2] * s;\
}\

//    _____                  
//   |_   _|  _ _ __  ___ ___
//     | || || | '_ \/ -_|_-<
//     |_| \_, | .__/\___/__/
//         |__/|_|           
typedef union{
	int16_t v[3];
	struct{
		int16_t x, y, z;
	};
} vec3i16_t;

typedef union{
	float v[3];
	struct{
		float x, y, z;
	};
} vec3f_t;

typedef struct{
	uint32_t waypoints;
} gpsRouteHeader_t;

typedef struct{
	vec3f_t location;
	float   tolerance;
	uint8_t nextWaypoint;
	uint8_t flags; 
} gpsWaypoint_t;

struct __GpsWaypoint;
typedef struct __GpsWaypoint{
	gpsWaypoint_t  self;
	struct __GpsWaypoint* next;
} gpsWaypointCont_t;

#endif