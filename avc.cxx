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
#include "timer.h"
#include "controls/servos.h"
#include "sensors/aggergate.h"
#include "decision/agents.h"
#include "utilities/diagnostics/diagnostics.h"

void error(const char* msg, ...)
{
	va_list argp;

	va_start(argp, msg);
	vprintf(msg, argp);
	va_end(argp);
}

void sigHandler(int sig)
{	
	if(sig == SIGINT){
		conSet(SERVO_STEERING, 50);
		conSet(SERVO_THROTTLE, 50);
		exit(0);
	}	
}

int main(int argc, char* argv[])
{
	int err = 0;
	
	// // start servo controlling
	err = conInit();
	if(err) return err;

	assert(signal(SIGINT, sigHandler) != SIG_ERR);

	// // start up IMU and GPS sensors
	err = senInit("/dev/i2c-1", "/dev/ttyAMA0", "./imu.cal");
	if(err){
		error("Initializing sensor '%s' failed (%d, %d)\n", NULL, err, errno);
		return err;
	}

	// sensors are started, start diagnostic endpoint
	diagHost(1340);

	// load a route
	err = gpsRouteLoad(argv[1], &SYS.route.start);
	if(err){
		error("Loading gps route '%s' failed (%d, %d)", argv[1], err, errno);
		return err;
	}
	SYS.route.currentWaypoint = SYS.route.start;

	// setup all the decision agents
	agentInitAgents();

	while(1){
		senUpdate(&SYS.body);

		AGENT_STEERING.action(NULL, NULL);
		AGENT_THROTTLE.action(NULL, NULL);

		timerUpdate();
	}

	return 0;
}

