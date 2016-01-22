#include <unistd.h>
#include <assert.h>
#include <indicurses.h>

#include "system.h"
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

	int res = senInit(argv[1], argv[2], "./../imu.cal");
	printf("Res %d\n", res);

	gpsWaypointCont_t* waypoints;
	assert(!gpsRouteLoad(argv[3], &waypoints));
	assert(!diagHost(1340));
	printf("Loaded GPS route!\n");
	for(gpsWaypointCont_t* w = waypoints; w; w = w->next){
		printf("Waypoint %d (%f, %f)\n", w->self.nextWaypoint, w->self.location.x, w->self.location.y);
	}

	float qtr = M_PI / 4.0f;
	vec3f_t headings[] = {
		normalAngle(qtr * 0), // N
		normalAngle(qtr * 1), // NW
		normalAngle(qtr * 2), // W
		normalAngle(qtr * 3), // SW
		normalAngle(qtr * 4), // S
		normalAngle(qtr * 5), // SE
		normalAngle(qtr * 6), // E
		normalAngle(qtr * 7), // NE
	};

	const char* cardinals[] = {
		"N",
		"NW",
		"W",
		"SW",
		"S",
		"SE",
		"E",
		"NE",
	};

	sleep(2);

	assert(!icInit());
	start_color();
	init_pair(1, COLOR_RED, COLOR_BLACK);
	init_pair(2, COLOR_BLUE, COLOR_BLACK);
	clear();

	assert(!res);

	const int samples = 50;
	int i = 0;
	int readings[3][50] = {{},{},{}};
	int origin[100];
	
	for(int i = 100; i--; origin[i] = 0)
		bzero(readings[0], sizeof(int) * samples);

	while(1){
		char buf[64] = {};
		senUpdate(&SYS.body);

		int topLeft[2] = { 5, 2 };
		int bottomRight[2] = { IC_TERM_WIDTH - 5, IC_TERM_HEIGHT - 2};	
		
		int center = (bottomRight[1] - topLeft[1]) / 2;

		const int scale = 50;

		readings[0][i] = center + SYS.body.imu.adjReadings.linear.y / 10;
		readings[1][i] = center + SYS.body.estimated.velocity.linear.y * 10;

		++i;
		i %= samples;

		clear();
		attron(COLOR_PAIR(1));
		icLineGraph(topLeft, bottomRight, 'r', readings[0], samples);
		attron(COLOR_PAIR(2));
		icLineGraph(topLeft, bottomRight, 'e', readings[1], samples);
		attroff(COLOR_PAIR(2));

		icLineGraph(topLeft, bottomRight, '-', origin, 100);

		vec3f_t acc = SYS.body.imu.adjReadings.linear;
		vec3f_t mag = SYS.body.imu.adjReadings.mag;
		vec3f_t gry = SYS.body.imu.adjReadings.rotational;
		sprintf(buf, "acc (%f, %f, %f) mag (%f, %f, %f) gyro (%f, %f, %f)", acc.x, acc.y, acc.z, mag.x, mag.y, mag.z, gry.x, gry.y, gry.z);
		icText(2, 2, buf);
		
		int bi = 0;
		float bd = vec3fDot(&SYS.body.measured.heading, headings);
		for(int i = 8; i--;){
			float d = vec3fDot(&SYS.body.measured.heading, headings + i);
			if(d > bd){
				bd = d;
				bi = i;
			}
		}

		sprintf(buf, "Compass: %s", cardinals[bi]);
		icText(2, 3, buf);

		icPresent();

		sysTimerUpdate();
		usleep(1000);
		icPresent();
	}

	return 0;
}
