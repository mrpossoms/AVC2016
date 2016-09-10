#include "scanner.h"
#include "gps.h"
#include "base/system.h"
#include <unistd.h>

static int SERVO_DIR = 1;
static float LAST_SCANNED;

int scn_init(
	scn_t* scanner,
	int servo_min,
	int servo_max,
	float servo_range,
	float sec_per_tick,
	float far_plane)
{
	if(!scanner) return -1;

	scanner->servo.min = servo_min;
	scanner->servo.max = servo_max;
	scanner->servo.range = servo_range;
	scanner->servo.rate = sec_per_tick;
	scanner->servo.position = (servo_min + servo_max) / 2;
	scanner->far_plane = far_plane;

	// set servo to home position
	ctrlSet(SERVO_SCANNER, scanner->servo.position);

	const float angle_tick = -scanner->servo.range / (float)SCANNER_RES;
	const float angle_0 = scanner->servo.range / 2;

	// assign angular values to each data sample
	for(int i = SCANNER_RES; i--;)
	{
		scanner->readings[i].angle = i * angle_tick + angle_0;
		scanner->readings[i].index = i;
		scanner->readings[i].distance = 1000;
	}

	return 0;
}
//------------------------------------------------------------------------------
static int _scn_obs_compare(const void* a, const void* b)
{
	scn_datum_t* A = (scn_datum_t*)a;
	scn_datum_t* B = (scn_datum_t*)b;

	if(A->distance < B->distance) return -1;
	if(A->distance > B->distance) return  1;
	return 0;
}
//------------------------------------------------------------------------------
int scn_find_obstacles(
	scn_t* scanner,
	scn_obstacle_t* list,
	int list_size)
{
	const float max_dd = 0.5; // max change in distance Meters
	int obs_ind = 0;

	scn_datum_t* last = scanner->readings;
	scn_datum_t* obs_start = last;
	float nearest_point = obs_start->distance;

	for(int i = 1; i < SCANNER_RES; ++i)
	{
		scn_datum_t* curr = scanner->readings + i;
	
		if(curr->time_taken == 0) continue;
		float dd = fabs(last->distance - curr->distance);

		// how big is the difference between this measurement, and the last?
		// is it too big?
		if(dd > max_dd || curr->index == SCANNER_RES - 1)
		{
			// define the width of it, with the left and right
			// indices
			scn_obstacle_t* obs = list + obs_ind;
			int s_i = obs->left_i  = obs_start->index;
			int e_i = obs->right_i = curr->index;
			obs->nearest = nearest_point;

			// compute the obstacle centroid
			vec3_add(obs->centroid.v, obs_start->location.v, last->location.v);
			vec3_scale(obs->centroid.v, obs->centroid.v, 0.5);

			// find the 'radius' of the obstacle
			vec3 delta;
			vec3_sub(delta, obs_start->location.v, curr->location.v);
			obs->radius = vec3_len(delta) / 2;

			// refresh the obs indices on the scanner data (for vis.)
			for(int j = s_i; j <= e_i; ++j)
			{
				scanner->readings[j].obs_ind = obs_ind;
			}

			++obs_ind;

			// have we run out of allocated obstacles?
			if(obs_ind >= list_size)
			{
				return -1;
			}

			// reset for the next obs
			obs_start = curr;
			nearest_point = curr->distance;
		}
		else{
			// watch for a nearer point
			if(curr->distance < nearest_point)
			{
				nearest_point = curr->distance;
			}
		}

		last = curr;
	}

	// sort obstacles, nearest to furthest
	qsort(list, list_size, sizeof(scn_obstacle_t), _scn_obs_compare);

	return 0;
}
//------------------------------------------------------------------------------
void scn_update(scn_t* scanner, float meters)
{
	int* pos = &scanner->servo.position;
	int min = scanner->servo.min, max = scanner->servo.max;
	int i = SCANNER_RES * (*pos - min) / (float)(max - min);
	scn_datum_t* last = scanner->last_reading;

	if(SYS.timeUp - LAST_SCANNED < scanner->servo.rate || meters < 0.1)
	{
		return;
	}

	float distance = meters;

	scn_datum_t* reading = scanner->readings + i;
	reading->time_taken = SYS.timeUp;


	if(distance > 40)
	{
		if(last)
		{
			reading->distance = last->distance;
		}
		else
		{
			reading->distance = distance;
		}
	}
	else
	{
		reading->distance = distance;
	}

	vec3f_t temp;
	quat rotation = { 0, 0, 0, 1 };
	vec3_scale(temp.v, SYS.pose.heading.v, (float)mtodeg(reading->distance)); // scale the normalized heading
	quat_from_axis_angle(rotation, 0, 1, 0, reading->angle);
	quat_mul_vec3(temp.v, rotation, temp.v); // rotate it by the angle of the measurement
	vec3Add(reading->location, temp, SYS.pose.pos); // add to current position

	scanner->last_reading = scanner->readings + i;
	*pos += SERVO_DIR;

	// if we reach either side of the range, change SERVO_DIRections
	if(*pos == min || *pos == max - 1)
	{
		SERVO_DIR = -SERVO_DIR;
	}

	// move the servo
	ctrlSet(SERVO_SCANNER, *pos);
	LAST_SCANNED = SYS.timeUp;

	scn_find_obstacles(scanner, scanner->obstacles, SCANNER_RES);
}
//------------------------------------------------------------------------------
int obs_pos_rel(scn_obstacle_t* a, scn_obstacle_t* b)
{
	if(a->left_i < b->left_i) return -1; // a is left of b
	if(a->right_i > b->right_i) return 1; // a is right of b
	return 0;// a is in-side of b
}
//------------------------------------------------------------------------------
int obs_intersect(scn_obstacle_t* obs, vec3f_t v0, vec3f_t v1, vec3f_t* res)
{
	vec3f_t v = { v1.x - v0.y, 0, v1.z - v0.z };
	vec3f_t d = { obs->centroid.x - v0.x, 0, obs->centroid.y - v1.z };

	float a = v.x + v.z;
	float b = 2 * (v.x * d.x + v.z * d.z);
	float c = vec3_mul_inner(d.v, d.v) - obs->radius;
	float rad_inner = b * b - 4 * a * c;

	if(a == 0 || rad_inner < 0) return 0; // can't intersect

	float radical = sqrtf(rad_inner);
	float base = 2 * a;
	float t[2] = { (-b + radical) / base, (-b - radical) / base };

	for(int i = 2; i--;){
		if(t[i] >= 0 && t[i] <= 1)
		{
			return 1; // intersecting
		}
	}

	return -1; // not intersecting
}
