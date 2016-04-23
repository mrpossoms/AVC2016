#include "system.h"
#include <sys/time.h>

static struct timeval START_TIME;
static float LAST_TIME_UP;

void sysTimerUpdate()
{
	static int isSetup;
	int64_t usElapsed = 0;

	// compute the elapsed time in seconds
	struct timeval now;
	gettimeofday(&now, NULL);
	
	if(!isSetup){
		usElapsed = (now.tv_sec - START_TIME.tv_sec) * 1000000 +
		                    (now.tv_usec - START_TIME.tv_usec);
	}
	else{
		START_TIME = now;
		isSetup = 1;
	}

	SYS.timeUp = usElapsed / 1000000.0;
	SYS.dt = SYS.timeUp - LAST_TIME_UP;

	LAST_TIME_UP = SYS.timeUp;
}
