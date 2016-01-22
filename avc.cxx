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

#include "system.h"
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

int main(int argc, char* argv[])
{
	int err = 0;
	openlog("AVC_BOT", 0, 0);	

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

	while(1){
		senUpdate(&SYS.body);

		AGENT_ROUTING.action(NULL, NULL);
		AGENT_STEERING.action(NULL, NULL);
		AGENT_THROTTLE.action(NULL, NULL);

		sysTimerUpdate();
	}

	return 0;
}

