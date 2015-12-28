#include <unistd.h>
#include <assert.h>
#include <indicurses.h>

#include "system.h"
#include "aggergate.h"
#include "timer.h"

int main(int argc, char* argv[])
{
	if(argc < 3){
		printf("Usage:\n[imu device][gps device]\n");
		return 1;
	}

	int res = senInit(argv[1], argv[2], "./../imu.cal");
	printf("Res %d\n", res);

	assert(!icInit());
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

		vec3f_t posMes = SYS.body.measured.position, posEst = SYS.body.estimated.position;
		vec3f_t velMes = SYS.body.measured.velocity, velEst = SYS.body.estimated.velocity;
	
		int topLeft[2] = { 5, 2 };
		int bottomRight[2] = { IC_TERM_WIDTH - 5, IC_TERM_HEIGHT - 2};	
		
		int center = (bottomRight[1] - topLeft[1]) / 2;

		const int scale = 500;

		readings[0][i] = center + SYS.body.imu.rawReadings.linear.y / scale;
		readings[1][i] = center + velEst.y / scale;

		++i;
		i %= samples;

		clear();
		icLineGraph(topLeft, bottomRight, '-', origin, 100);
		icLineGraph(topLeft, bottomRight, 'r', readings[0], samples);
		icLineGraph(topLeft, bottomRight, 'e', readings[1], samples);

		icPresent();

		timerUpdate();
		usleep(1000);
		icPresent();
	}

	return 0;
}
