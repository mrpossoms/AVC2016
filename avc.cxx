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
#include <pthread.h>

#include "base/system.h"
#include "controls/servos.h"
#include "sensors/aggergate.h"
#include "decision/agents.h"
#include "utilities/diagnostics/diagnostics.h"
#include "utilities/RC/rc.h"

pthread_t RC_THREAD;
static char** ARGV;
static int    ARGC;

static void mark_process()
{
	int fd;
	pid_t id = getpid();
	char path[32] = {};

	sprintf(path, "%d.pid", id);
	fd = open(path, O_CREAT | O_TRUNC, O_WRONLY);
	close(fd);
}
//------------------------------------------------------------------------------
static void unmark_process()
{
	pid_t id = getpid();
	char path[32] = {};

	sprintf(path, "%d.pid", id);
	unlink(path);
}
//------------------------------------------------------------------------------
void sigHandler(int sig)
{
	if(sig == SIGINT || sig == SIGKILL || sig == SIGTERM){
		ctrlSet(SERVO_STEERING, 50);
		ctrlSet(SERVO_THROTTLE, 50);
		unmark_process();
		exit(0);
	}
}
//------------------------------------------------------------------------------
void* RCHandler(void* arg)
{
	while(1){
		rcComm();
	}
}
//------------------------------------------------------------------------------
int hasOpt(const char* target)
{
	for(int i = ARGC; i--;){
		if(!strcmp(ARGV[i], target)) return 1;
	}

	return 0;
}
//------------------------------------------------------------------------------
int intFromOpt(const char* target, int* val)
{
	for(int i = argc; i--;){
		if(!strncmp(argv[i], target, strlen(target))){
			char* num = argv[i];
			for(int j = 0; argv[i][j++] != '='; num++);
			*val = atoi(num + 1);
			return 0;
		}
	}

	return -1;
}
//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	int err = 0, isRC = 0;

	ARGC = argc; ARGV = argv;
	openlog("AVC_BOT", 0, 0);
	mark_process();

	if(hasOpt("--RC")){
		printf("Using radio control\n");
		isRC = 1;
		pthread_create(&RC_THREAD, NULL, RCHandler, NULL);
	}


	SYS.debugging = hasOpt("--debug");
	SYS.magCal = hasOpt("--mag-cal");
	SYS.following = hasOpt("--follow");

	if(SYS.following){
		SYS.shm = sysAttachSHM(); 
	}

	if(intFromOpt("--speed", &SYS.maxSpeed)){
		SYS.maxSpeed = 53;
		printf("No speed set\n");
	}

	printf("Max speed: %d\n", SYS.maxSpeed);

	// start servo controlling
	if(!hasOpt("--no-servo")){
		err = ctrlInit();
		if(err){
			SYS_ERR("Starting servo driver failed %d", err);
			return err;
		}
	}

	if(signal(SIGINT, sigHandler) == SIG_ERR){
		SYS_ERR("Registering signal SIGINT failed %d", errno);
		return err;
	}

	// start up IMU and GPS sensors
	err = senInit("/dev/i2c-1", "/dev/ttyAMA0", "./imu.cal");
	if(err){
		SYS_ERR("Initializing sensor '%s' failed", "");
		return err;
	}

	if(hasOpt("--mag-reset")){
		printf("Reseting magnetometer calibration readings\n");
		bzero(SYS.body.imu.calMinMax[0].mag.v, sizeof(vec3i16_t));
		bzero(SYS.body.imu.calMinMax[1].mag.v, sizeof(vec3i16_t));
	}

	// sensors are started, start diagnostic endpoint
	diagHost(1340);

	// load a route
	if(argc >= 2){
		printf("Loading route...");
		err = gpsRouteLoad(argv[1], &SYS.route.start);
		if(err){
			printf("error\n");
			SYS_ERR("Loading gps route '%s' failed", argv[1]);
			return err;
		}
		SYS.route.currentWaypoint = SYS.route.start;
		printf("OK\n");
	}
	else{
		printf("No route loaded\n");
	}

	// setup all the decision agents
	agentInitAgents();

	int useBlackBox = !hasOpt("--no-black-box");
	printf("Starting main loop\n");
	while(1){
		senUpdate(&SYS.body);

		if(!isRC){
			AGENT_ROUTING.action(NULL, NULL);
			AGENT_STEERING.action(NULL, NULL);

			if(hasOpt("--use-throttle")){
				AGENT_THROTTLE.action(NULL, NULL);
			}
			AGENT_CRASH_DETECTOR.action(NULL, NULL);
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

		//usleep(1000);
	}

	sigHandler(SIGINT);
	return 0;
}
