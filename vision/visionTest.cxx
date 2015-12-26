#include <unistd.h>
#include <stdlib.h>

#include "system.h"
#include "vision.h"

int main(int argc, char* argv[])
{
	int w = 128, h=128;
	trackingState_t tracking = {};

	if(argc < 3){
		printf("Usage:\n-w[width] -h[height]\n");
		return 1;
	}

	for(int i = 1; i < argc; ++i){
		if(!strncmp(argv[i], "-w", 2)){
			w = atoi(argv[i] + 2);
		}
		if(!strncmp(argv[i], "-h", 2)){
			h = atoi(argv[i] + 2);
		}
	}

	visionInit(&tracking, w, h);

	while(1){
		visionUpdate(&tracking);
	}

	return 0;
}