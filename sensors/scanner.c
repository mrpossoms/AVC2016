#include "scanner.h"
#include "base/system.h"
#include "i2c.h"
#include <unistd.h>
#include <arpa/inet.h>

#define HIST_SIZE 10
#define SCN_I2C_ADDR 0x62

static int I2C_FD;

static int SERVO_DIR = 1;
static float LAST_SCANNED;

int scn_init(
	scn_t* scanner,
	int i2c_dev,
	int servo_min,
	int servo_max,
	float servo_range,
	float sec_per_tick,
	float far_plane)
{
	if(!scanner) return -1;
	if(i2c_dev < 0) return -2;

	I2C_FD = i2c_dev;


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
		float dd = fabs(last->distance - curr->distance);

		// how big is the difference between this measurement, and the last?
		// is it too big?
		if(dd > max_dd)
		{
			// define the width of it, with the left and right
			// indices
			int s_i = list[obs_ind].left_i  = obs_start->index;
			int e_i = list[obs_ind].right_i = curr->index;
			list[obs_ind].nearest = nearest_point;

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

	if(SYS.timeUp - LAST_SCANNED < scanner->servo.rate)
	{
		return;
	}

	float distance = meters;

	// TODO i is the issue here. I is servo position where really we want index
	if(distance > 40)
	{
		scanner->readings[i].time_taken = SYS.timeUp;

		if(last)
		{
			scanner->readings[i].distance = last->distance;
		}
		else
		{
			scanner->readings[i].distance = distance;
		}
	}
	else
	{
		scanner->readings[i].time_taken = SYS.timeUp;

/*
		if(last)
		{
			scanner->readings[i].distance = distance * .75 + last->distance * .25f;
		}
		else
*/
		{
			scanner->readings[i].distance = distance;
		}
	}

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
