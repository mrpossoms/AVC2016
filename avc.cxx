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
#include <dirent.h>
#include <opt.h>

#include "base/system.h"
#include "controls/servos.h"
#include "sensors/aggergate.h"
#include "decision/agents.h"
#include "utilities/diagnostics/diagnostics.h"
#include "utilities/RC/rc.h"

const char HEADER[] = "  _  ___   _ ___  _______  ___ _____ \n"
                      " | |/ / | | | _ \\|_  / _ )/ _ \\_   _|\n"
                      " | ' <| |_| |   / / /| _ \\ (_) || |  \n"
                      " |_|\\_\\\\___/|_|_\\/___|___/\\___/ |_|  \n"
                      "                                     \n";

pthread_t RC_THREAD;
int MISSION_FD = 0;
uint32_t MISSION_WAYPOINTS = 0;
int IS_RC = 0, REC_ROUTE = 0;
int USE_BLACK_BOX = 1;
int WAIT_TIME;
int HAS_STARTED = 0;

void* RCHandler(void* arg);

//      _   ___  ___   _  _         _ _
//     /_\ | _ \/ __| | || |_ _  __| | |_ _ ___
//    / _ \|   / (_ | | __ | ' \/ _` | | '_(_-<_
//   /_/ \_\_|_\\___| |_||_|_||_\__,_|_|_| /__(_)
//
static void start_now_opt(char* value, int present)
{
	HAS_STARTED = present;
}

static void mag_reset_opt(char* value, int present)
{
	if(present){
		printf("Reseting magnetometer calibration readings\n");
		bzero(SYS.sensors.imu.calMinMax[0].mag.v, sizeof(vec3i16_t));
		bzero(SYS.sensors.imu.calMinMax[1].mag.v, sizeof(vec3i16_t));
		SYS.magCal = 1;
	}
}

static void delay_opt(char* value, int present)
{
	if(present)
	{
		WAIT_TIME = atoi(value);
	}
}

static void speed_opt(char* value, int present)
{
	if(present)
	{
		SYS.maxSpeed = atoi(value);
	}
	else
	{
		SYS.maxSpeed = 50;
		printf("No speed set\n");
	}

	printf("Max speed: %d\n", SYS.maxSpeed);
}

static void no_sensors_opt(char* value, int present)
{
	int err = 0;
	if(present){
		err = senInit("/dev/null", "/dev/null", "./imu.cal");
	}
	else{
		err = senInit("/dev/i2c-1", "/dev/ttyAMA0", "./imu.cal");
	}

	if(err){
		SYS_ERR("Initializing sensor '%s' failed", "");
		exit(err);
	}

}

static void no_servos_opt(char* value, int present)
{
	if(present) return;

	int err = ctrlInit();
	if(err){
		SYS_ERR("Starting servo driver failed %d", err);
		exit(err);
	}
}

static void record_opt(char* value, int present)
{
	REC_ROUTE = IS_RC && present;
}

static void rc_opt(char* value, int present)
{
	IS_RC = present;

	if(IS_RC)
	{
		printf("Using radio control\n");
		pthread_create(&RC_THREAD, NULL, RCHandler, NULL);
	}
}

static void no_scanner_opt(char* value, int present)
{
	SYS.use_scanner = !present;
}
//------------------------------------------------------------------------------
static void mark_process()
{
	int fd;
	pid_t id = getpid();
	char path[32] = {};

	snprintf(path, sizeof(path), "%d.pid", id);
	fd = open(path, O_CREAT | O_TRUNC, O_WRONLY);
	close(fd);
}
//------------------------------------------------------------------------------
static void unmark_process()
{
	char path[32] = {};

	DIR* dir = opendir("./");

	if(dir){
		struct dirent* ep = NULL;
		while((ep = readdir(dir))){
			if(strstr(ep->d_name, ".pid")){
				unlink(ep->d_name);
			}
		}
	}
}
//------------------------------------------------------------------------------
void sigHandler(int sig)
{
	ctrlSet(SERVO_STEERING, 50);
	ctrlSet(SERVO_THROTTLE, 50);
	unmark_process();
	if(MISSION_FD > 0){
		gpsRouteHeader_t hdr = { MISSION_WAYPOINTS };
		printf("%d waypoints\n", MISSION_WAYPOINTS);
		lseek(MISSION_FD, 0, SEEK_SET);
		write(MISSION_FD, &hdr, sizeof(hdr));
		close(MISSION_FD);
		fflush(stdout);
	}
	exit(0);
}
//------------------------------------------------------------------------------
void* RCHandler(void* arg)
{
	while(1){
		rcComm();
	}
}
//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	USE_OPT

	int err = 0;
	openlog("AVC_BOT", 0, 0);
	mark_process();

	USE_BLACK_BOX = opt_has_flag("--no-black-box");
	SYS.debugging = opt_has_flag("--debug");
	SYS.magCal = opt_has_flag("--mag-cal");
	SYS.following = opt_has_flag("--follow");
	assert(!SYS.following);

	OPT_LIST_START
	{
		"--start-now",
		"",
		0,
		start_now_opt
	},
	{
		"--record",
		"Records over top of existing mission file",
		0,
		record_opt
	},
	{
		"--RC",
		"Enables remote control of throttle and steering.",
		0,
		rc_opt
	},
	{
		"--mag-reset",
		"Zeros out all mag calibration values",
		0,
		mag_reset_opt
	},
	{
		"--no-sensors",
		"Skips enabling sensors.",
		0,
		no_sensors_opt
	},
	{
		"--speed",
		"Set max speed. 50 is stopped.",
		1,
		speed_opt
	},

	{
		"--no-servos",
		"Skips starting servo driver and enabling servo control system",
		0,
		no_servos_opt
	},
	{
		"--delay",
		"Causes a delay of N seconds after initialization",
		1,
		delay_opt
	},
	{
		"--no-scanner",
		"",
		0,
		no_scanner_opt
	}
	OPT_LIST_END(HEADER)

	int signals[] = { SIGINT, SIGTERM, SIGHUP };
	struct sigaction sa = {};
	sa.sa_handler = &sigHandler;
	sa.sa_flags = SA_RESTART;
	// Block every signal during the handler
	sigfillset(&sa.sa_mask);

	for(int i = sizeof(signals) / sizeof(int); i--;){
		if(sigaction(signals[i], &sa, NULL) == -1){
			SYS_ERR("Registering signal %d failed %d", signals[i], errno);
			return err;
		}
	}

	// sensors are started, start diagnostic endpoint
	diagHost(1340);

	// load a route
	if(argc >= 2 && !REC_ROUTE){
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

		if(REC_ROUTE){
			static vec3f_t last_pos;
			gpsRouteHeader_t hdr;
			MISSION_FD = open("mission.gps", O_WRONLY | O_TRUNC | O_CREAT, 0666);
			if(MISSION_FD < 0){
				SYS_ERR("Failed to open '%s'\n", "mission.gps");
				return -1;
			}

			// seek to the starting position of the first waypoint
			lseek(MISSION_FD, sizeof(hdr), SEEK_SET);
		}
	}

	if(SYS.following){
		SYS.shm = sysAttachSHM();

		if(!SYS.shm){
			SYS_ERR("Attaching to shared memory failed %s\n", "");
			return -1;
		}

		SYS.route.currentWaypoint = (gpsWaypointCont_t*)(&SYS.shm->followLocation);
	}

	// setup all the decision agents
	agentInitAgents();

	SYS.pose.pos.x = SYS.pose.pos.y = 1;

	printf("exp %f\n", SYS.sensors.mag_expected.len);
	
	sleep(WAIT_TIME);
	printf("Starting main loop\n");
	uint16_t last_wp_idx = 0;
	while(1){
		assert(!isnan(SYS.pose.pos.x));
		senUpdate(&SYS.sensors);
		assert(!isnan(SYS.pose.pos.x));

		if(!HAS_STARTED && fabs(SYS.sensors.measured.acc.y) > LIL_G / 4)
		{
			printf("Lets go!\n");
			HAS_STARTED = 1;
		}

		if(!IS_RC){
			// TODO
			SYS.sensors.hasGpsFix = 1;

			if(HAS_STARTED)
			{
				AGENT_ROUTING.action(NULL, NULL);
				AGENT_STEERING.action(NULL, NULL);
				AGENT_THROTTLE.action(NULL, NULL);
				AGENT_CRASH_DETECTOR.action(NULL, NULL);
			}		

			// if there is no next goal or GPS then terminate
			if(!SYS.route.currentWaypoint){
				//break;
			}
		}
		else if(REC_ROUTE)
		{
			static vec3d_t last_pos;
			gpsWaypoint_t wp = {
				.location = SYS.pose.pos,
				.index = last_wp_idx,
			};

			float w = fabs(SYS.sensors.measured.gyro.z);
			const float min_dist = mtodeg(1);
			if(vec3Dist(wp.location, last_pos) > min_dist && w >= 0.08){
				printf("Saving pos %f, %f\n", wp.location.x, wp.location.y);
				write(MISSION_FD, &wp, sizeof(gpsWaypoint_t));
				last_pos = wp.location;
				MISSION_WAYPOINTS++;
				last_wp_idx++;
			}
		}

		sysTimerUpdate();

		// record system state, if indicated
		if(USE_BLACK_BOX){
			diagBlkBoxLog();
		}

	}

	sigHandler(SIGINT);
	return 0;
}
