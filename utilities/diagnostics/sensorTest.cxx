#include <unistd.h>
#include <assert.h>
#include <indicurses.h>

#include "base/system.h"
#include "aggergate.h"
#include "utilities/diagnostics/diagnostics.h"

vec3f_t normalAngle(float angle)
{
	vec3f_t n = { cos(angle), sin(angle), 0 };
	return n;
}

int main(int argc, char* argv[])
{
	if(argc < 4){
		printf("Usage:\n[imu device][gps device][gps mission]\n");
		return 1;
	}

	int res = senInit(argv[1], argv[2], "./../../imu.cal");
	printf("Res %d\n", res);
	assert(!res);
	
	gpsWaypointCont_t* waypoints;
	assert(!gpsRouteLoad(argv[3], &waypoints));
	assert(!diagHost(1340));
	printf("Loaded GPS route!\n");
	for(gpsWaypointCont_t* w = waypoints; w; w = w->next){
		printf("Waypoint %d (%f, %f)\n", w->self.nextWaypoint, w->self.location.x, w->self.location.y);
	}

	sleep(2);

	assert(!icInit());
	start_color();
	init_pair(1, COLOR_RED, COLOR_BLACK);
	init_pair(2, COLOR_BLUE, COLOR_BLACK);
	init_pair(3, COLOR_GREEN, COLOR_BLACK);
	clear();

	const int samples = 50;
	int i = 0;
	int readings[3][50] = {{},{},{}};
	int origin[100] = {};
	//int minMax[2] = { -32000, 32000 };
	int minMax[2] = { -20, 20 };

	bzero(readings[0], sizeof(int) * samples);

	while(1){
		char buf[64] = {};
		senUpdate(&SYS.body);

		int topLeft[2] = { 5, 2 };
		int bottomRight[2] = { IC_TERM_WIDTH - 5, IC_TERM_HEIGHT - 2};

		readings[0][i] = SYS.body.imu.cal.gyro.z * 10;
		readings[1][i] = SYS.body.imu.cal.acc.y;
		readings[2][i] = SYS.body.imu.cal.acc.z;

		++i;
		i %= samples;

		clear();
		attron(COLOR_PAIR(1));
		icLineGraph(topLeft, bottomRight, 'r', readings[0], samples, minMax);
		
		attron(COLOR_PAIR(2));
		icLineGraph(topLeft, bottomRight, 'y', readings[1], samples, minMax);

		attron(COLOR_PAIR(3));
		icLineGraph(topLeft, bottomRight, 'z', readings[2], samples, minMax);

		attroff(COLOR_PAIR(3));
		icLineGraph(topLeft, bottomRight, '-', origin, 10, minMax);

		vec3f_t acc = SYS.body.imu.cal.acc;
		vec3f_t mag = SYS.body.imu.cal.mag;
		vec3f_t gry = SYS.body.imu.cal.gyro;
		sensorStatei_t *m = SYS.body.imu.calMinMax, *M = SYS.body.imu.calMinMax + 1;
		icTextf(IC_TERM_WIDTH - 60, 2, "acc (%f, %f, %f)\nmag (%f, %f, %f)\ngyro (%f, %f, %f)", acc.x, acc.y, acc.z, mag.x, mag.y, mag.z, gry.x, gry.y, gry.z);
		icTextf(IC_TERM_WIDTH - 60, 8, "GPS loc (%f, %f)", SYS.body.measured.position.x, SYS.body.measured.position.y);

		icPresent();

		sysTimerUpdate();
		usleep(1000);
		icPresent();
	}

	return 0;
}
