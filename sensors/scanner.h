#ifndef AVC_SCANNER
#define AVC_SCANNER

#include "controls/servos.h"

#ifndef __cplusplus
extern "C" {
#endif

#define SCANNER_RES 100

typedef struct {
	float time_taken;
	float distance;
	float angle;
} scn_datum_t;

typedef struct {
	vec3f_t left, right;
	vec3f_t nearest;
} scn_obstacle_t;

typedef struct {
	struct {
		float range;
		float rate;
		int min, max;
		int position;
	} servo;
	float far_plane;
	scn_datum_t readings[SCANNER_RES];
} scn_t;

int scn_init(
	scn_t* scanner,
	int i2c_dev,
	int servo_min,
	int servo_max,
	float servo_range,
	float sec_per_tick);

void scn_find_obstacles(scn_obstacle_t* list, int list_size);

#ifndef __cplusplus
}
#endif

#endif
