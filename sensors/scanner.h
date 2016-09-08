#ifndef AVC_SCANNER
#define AVC_SCANNER

#include <inttypes.h>
#include "controls/servos.h"
#include "base/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SCANNER_RES 30

typedef struct {
	float time_taken;
	float distance;
	float angle;
	uint8_t index;
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
	scn_datum_t* last_reading;
} scn_t;

int scn_init(
	scn_t* scanner,
	int i2c_dev,
	int servo_min,
	int servo_max,
	float servo_range,
	float sec_per_tick,
	float far_plane);

void scn_update(scn_t* scanner, float meters);

void scn_find_obstacles(
	scn_t* scanner,
	scn_obstacle_t* list,
	int list_size);

#ifdef __cplusplus
}
#endif

#endif
