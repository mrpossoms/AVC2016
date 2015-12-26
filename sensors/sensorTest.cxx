#include <unistd.h>
#include <assert.h>

#include "system.h"
#include "aggergate.h"

int main(int argc, char* argv[])
{
	if(argc < 3){
		printf("Usage:\n[imu device][gps device]\n");
		return 1;
	}

	int res = senInit(argv[1], argv[2], "./../imu.cal");
	printf("Res %d\n", res);

	assert(!res);

	while(1){
		printf("Begin\n");
		senUpdate(&SYS.body);

		vec3f_t *posMes = &SYS.body.measured.position, *posEst = &SYS.body.estimated.position;
		vec3f_t *velMes = &SYS.body.measured.velocity, *velEst = &SYS.body.estimated.velocity;
		printf(
			"Position mes:(%f, %f, %f) est:(%f, %f, %f)\n",
			posMes->x, posMes->y, posMes->z,
			posEst->x, posEst->y, posEst->z
		);
		printf(
			"Velocity mes:(%f, %f, %f) est:(%f, %f, %f)\n",
			velMes->x, velMes->y, velMes->z,
			velEst->x, velEst->y, velEst->z
		);

		usleep(1000);
	}

	return 0;
}
