#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

#include "system.h"
#include "timer.h"
#include "controls/servos.h"
#include "sensors/aggergate.h"
#include "decision/agents.h"
#include "utilities/diagnostics/diagnostics.h"

int main(int argc, char* argv[])
{
	int err = 0;

	// // start servo controlling
	// err = conInit();
	// if(err) return err;

	// // start up IMU and GPS sensors
	// err = senInit("/dev/i2c-1", "/dev/ttyAMA0", "./imu.cal");
	// if(err) return err;

	// sensors are started, start diagnostic endpoint
	diagHost(1340);

	// load a route
	err = gpsRouteLoad(argv[1], &SYS.route.start);
	if(err) return err;
	SYS.route.currentWaypoint = SYS.route.start;

	// setup all the decision agents
	agentInitAgents();

	while(1){
		// senUpdate(&SYS.body);

		AGENT_STEERING.action(NULL, NULL);

		timerUpdate();
	}

	return 0;
}

