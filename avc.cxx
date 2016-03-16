#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>

#include "base/system.h"
#include "controls/servos.h"
#include "sensors/aggergate.h"
#include "decision/agents.h"
#include "utilities/diagnostics/diagnostics.h"

void sigHandler(int sig)
{
	if(sig == SIGINT){
		ctrlSet(SERVO_STEERING, 50);
		ctrlSet(SERVO_THROTTLE, 50);
		exit(0);
	}
}

int hasOpt(char* argv[], int argc, const char* target){
	for(int i = argc; i--;){
		if(!strcmp(argv[i], target)) return 1;
	}

	return 0;
}

int main(int argc, char* argv[])
{
	int err = 0;
	openlog("AVC_BOT", 0, 0);

	if(hasOpt(argv, argc, "--debug")){
		SYS.debugging = 1;
	}

	if(hasOpt(argv, argc, "--mag-cal")){
		SYS.magCal = 1;
	}

	// // start servo controlling
	err = ctrlInit();
	if(err){
		SYS_ERR("Starting servo driver failed %d", err);
		return err;
	}

	if(signal(SIGINT, sigHandler) == SIG_ERR){
		SYS_ERR("Registering signal SIGINT failed", NULL);
		return err;
	}

	// // start up IMU and GPS sensors
	err = senInit("/dev/i2c-1", "/dev/ttyAMA0", "./imu.cal");
	if(err){
		SYS_ERR("Initializing sensor '%s' failed", "");
		return err;
	}

	// sensors are started, start diagnostic endpoint
	diagHost(1340);

	// load a route
	if(argc >= 2){
		err = gpsRouteLoad(argv[1], &SYS.route.start);
		if(err){
			SYS_ERR("Loading gps route '%s' failed", argv[1]);
			return err;
		}
		SYS.route.currentWaypoint = SYS.route.start;
	}
	else{
		printf("No route loaded\n");
	}

	// setup all the decision agents
	agentInitAgents();
	int useBlackBox = !hasOpt(argv, argc, "--no-black-box");
	while(1){
		senUpdate(&SYS.body);

		AGENT_ROUTING.action(NULL, NULL);
		AGENT_STEERING.action(NULL, NULL);

		if(hasOpt(argv, argc, "--use-throttle")){
			AGENT_THROTTLE.action(NULL, NULL);
		}
		sysTimerUpdate();

		// record system state, if indicated
		if(useBlackBox){
			diagBlkBoxLog();
		}

		// if there is no next goal or GPS then terminate
		if(!SYS.route.currentWaypoint){
			break;
		}

		usleep(1000);
	}

	sigHandler(SIGINT);
	return 0;
}
