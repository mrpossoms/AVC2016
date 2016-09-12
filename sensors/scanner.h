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
	vec3f_t location;
	float time_taken;
	float distance;
	float angle;
	uint8_t index, obs_ind;
} scn_datum_t;

typedef struct {
	uint8_t left_i, right_i, valid;
	vec3f_t centroid;
	float width;
	float nearest;
	float radius;
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
	scn_obstacle_t obstacles[SCANNER_RES];
	scn_datum_t* last_reading;
} scn_t;

int scn_init(
	scn_t* scanner,
	int servo_min,
	int servo_max,
	float servo_range,
	float sec_per_tick,
	float far_plane);

void scn_update(scn_t* scanner, float meters);

int scn_find_obstacles(
	scn_t* scanner,
	scn_obstacle_t* list,
	int list_size);

int scn_all_far(scn_t* scanner);

int obs_pos_rel(scn_obstacle_t* a, scn_obstacle_t* b);
int obs_intersect(scn_obstacle_t* obs, vec3f_t v0, vec3f_t v1, vec3f_t* res);
int obs_on_border(scn_obstacle_t* obs);
void obs_print_info(scn_obstacle_t* obs);

#ifdef __cplusplus
}
#endif

#endif
