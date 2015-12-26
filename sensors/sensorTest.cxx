#include <unistd.h>

#include "system.h"
#include "aggergate.h"

int main(int argc, char* argv[])
{
	if(argc < 3){
		printf("Usage:\n[imu device][gps device]\n");
		return 1;
	}

	senInit(argv[1], argv[2]);

	while(1){
		senUpdate(&SYS.body);
		usleep(1000);
	}

	return 0;
}