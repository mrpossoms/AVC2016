#include "scanner.h"
#include "base/system.h"
#include <pthread.h>
#include <unistd.h>

#define HIST_SIZE 10
#define SCN_I2C_ADDR 0xC4

pthread_t SWEEPER_THREAD;
static int I2C_FD;

static float _scn_get_range(scn_t* scanner)
{
	i2cSendByte(I2C_FD, SCN_I2C_ADDR, 0x0, 0x4);
	uint16_t dist = 0;

	while(i2cReqBytes(I2C_FD, SCN_I2C_ADDR, 0x0f, &dist, 2)) usleep(20000); 	 

	return dist / 100.f; // cm to meters
}
//------------------------------------------------------------------------------
void* _scn_sweeper_handler(void* args)
{
	scn_t* scanner = (scn_t*)args;
	float last_scanned = SYS.timeUp;
	int* i = &scanner->servo.position;
	int dir = 1;
	int min = scanner->servo.min, max = scanner->servo.max;

	while(1)
	{
		if(SYS.timeUp - last_scanned < scanner->servo.rate)
		{
			usleep(1000);
			continue;
		}

		scanner->readings[*i].time_take = SYS.timeUp;
		scanner->readings[*i].distance = _scn_get_range(scanner);

		*i += dir;

		// if we reach either side of the range, change directions
		if(*i == min || *i == max - 1)
		{
			dir = -dir;
		}
		
		// move the servo
		ctrlSet(SERVO_SCANNER, *i);
		last_scanned = SYS.timeUp;
	}

	return NULL;
}
//------------------------------------------------------------------------------
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
	ctrlSet(SERVO_SCANNER, scanner->servo.position);

	const float angle_tick = -scanner->servo.range / (float)SCANNER_RES;
	const float angle_0 = scanner->servo.range / 2;
	
	// assign angular values to each data sample
	for(int i = SCANNER_RES; i--;)
	{
		scanner->readings[i].angle = i * angle_tick + angle_0;
	}

	// wait for servo to make it to home. Not a huge fan of this
	sleep(1);

	return pthread_create(&SWEEPER_THREAD, NULL, NULL, scanner);
} 
//------------------------------------------------------------------------------
static int datum_comp(void* a, void* b)
{
	scn_datum_t* A = (scn_datum_t*)a;
	scn_datum_t* B = (scn_datum_t*)b;

	return A->distance < B->distance;
}
//------------------------------------------------------------------------------
static void _fill_hist(scn_datum_t*** hist, scn_t* scanner)
{
	int bucket_size[SCANNER_RES] = {};
	float hist_scale = HIST_SIZE / scanner->far_plane;

	for(int i = 0; i < SCANNER_RES; ++i)
	{
		scn_datum_t* d = scanner->readings + i;
		if(d->distance >= scanner->far_plane) continue;

		int bucket = (int)(hist_scale * d->distance);

		hist[bucket][bucket_size[bucket]] = d;
		++bucket_size[bucket];
	}
}
//------------------------------------------------------------------------------
void scn_find_obstacles(
	scn_t* scanner,
	scn_obstacle_t* list,
	int list_size)
{
	vec3f_t left = {}, right = {};
	vec3f_t heading = vec3fNorm(&SYS.pose.heading);
	scn_datum_t* data[SCANNER_RES] = {};
	scn_datum_t* hist[HIST_SIZE][SCANNER_RES] = {}
	
	_fill_hist(hist, scanner);

	// sort 
	//memcpy(data, scanner->readings, sizeof(scn_datum_t) * SCANNER_RES);
	//qsort(data, SCANNER_RES, sizeof(scn_datum_t), datum_comp);

	for(int i = 0; i < HIST_SIZE; ++i)
	{
		scn_datum_t* d = scanner->readings + i;
		

		
	}
}
