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

// File descriptors
int FD_IMU;

int main(int argc, char* argv[])
{
	// start servo controlling
	int err = conInit();
	if(err) return err;

	// start up IMU and GPS sensors
	err = senInit("/dev/i2c-1", "/dev/ttyAMA0", "./imu.cal");
	if(err) return err;

	// setup all the decision agents
	agentInitAgents();

	while(1){
		senUpdate(&SYS.body);

		

		timerUpdate();
	}

	return 0;
}

