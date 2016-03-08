#ifndef AVC_TYPES
#define AVC_TYPES

#include <inttypes.h>
#include <math.h>

#define vec2Sub(r, v1, v2){\
	(r).v[0] = (v1).v[0] - (v2).v[0];\
	(r).v[1] = (v1).v[1] - (v2).v[1];\
}\

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

#define vec3Lerp(r, a, b, p){\
	(r).v[0] = (a).v[0] * (1.0f - p) + (b).v[0] * p;\
	(r).v[1] = (a).v[1] * (1.0f - p) + (b).v[1] * p;\
	(r).v[2] = (a).v[2] * (1.0f - p) + (b).v[2] * p;\
}\

#define vec3Dist(a, b) sqrt(pow((a).x - (b).x, 2) + pow((a).y - (b).y, 2) + pow((a).z - (b).z, 2))

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
	float v[2];
	struct{
		float x, y;
	};
} vec2f_t;

typedef union{
	float v[3];
	struct{
		float x, y, z;
	};
} vec3f_t;

typedef enum{
	MISS_SRV_UPLOAD = 0,
	MISS_SRV_RUN,
	MISS_SRV_BLKBOX_LIST,
	MISS_SRV_BLKBOX_DOWNLOAD
} missSrvAct_t;

typedef struct{
   uint8_t name[64];
   uint32_t bytes;
} blkboxLog_t;

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

inline int vec3fIsNan(vec3f_t* v)
{
	return isnan(v->x) || isnan(v->y) || isnan(v->z);
}

inline vec3f_t vec3fSub(vec3f_t* v1, vec3f_t* v2)
{
	vec3f_t res = {
		v1->x - v2->x,
		v1->y - v2->y,
		v1->z - v2->z,
	};
	return res;
}

inline vec3f_t vec3fScl(vec3f_t* v, float s)
{
	vec3f_t res = {
		v->x * s,
		v->y * s,
		v->z * s,
	};
	return res;
}

inline vec3f_t vec3fMul(vec3f_t* v1, vec3f_t* v2)
{
	vec3f_t res = {
		v1->x * v2->x,
		v1->y * v2->y,
		v1->z * v2->z,
	};
	return res;
}

inline float vec3fDot(vec3f_t* v1, vec3f_t* v2)
{
	return v1->x * v2->x + v1->y * v2->y +v1->z * v2->z;
}

inline float vec3fMag(vec3f_t* v)
{
	return sqrt(v->x * v->x + v->y * v->y);
}

inline vec3f_t vec3fNorm(vec3f_t* v)
{
	float mag = vec3fMag(v);

	vec3f_t res = {
		v->x / mag,
		v->y / mag,
		v->z / mag,
	};
	return res;
}

inline void vec2fRot(vec2f_t* r, vec2f_t* v, float theta)
{
	float c = cos(theta), s = sin(theta);
	r->x = c * v->x - s * v->y;
	r->y = s * v->x + c * v->y;
}

inline float vec3fAng(vec3f_t* a, vec3f_t* b)
{
	vec3f_t an = vec3fNorm(a);
	vec3f_t bn = vec3fNorm(b);
	return acosf(vec3fDot(&an, &bn));
}

#endif
