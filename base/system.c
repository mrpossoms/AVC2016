#include <sys/shm.h>
#include "system.h"
#include "servos.h"

//     ___ _     _          _
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//
system_t SYS = {};

sysSnap_t sysSnapshot(system_t* sys)
{
	sysSnap_t snap = {};

	snap.pose    = sys->pose;
	snap.sensors.filtered = sys->sensors.filtered;
	snap.controls.throttle = ctrlGet(SERVO_THROTTLE);
	snap.controls.steering = ctrlGet(SERVO_STEERING);

	if(sys->sensors.scanner.last_reading)
	{
		snap.lastDepth = *sys->sensors.scanner.last_reading;
		snap.nearest_obs = sys->sensors.scanner.obstacles[0];
	}

	gpsWaypointCont_t* waypoint = sys->route.currentWaypoint;
	for(int i = 0; i < 10; ++i)
	{
		snap.waypoints[i] = waypoint->self;

		if(!sys->following && waypoint->next)
		{
			waypoint = waypoint->next;
		}
		else
		{
			break;
		}
	}

	return snap;
}
//---------------------------------------------------------------------------
void sysPrintSnap(sysSnap_t* snap)
{
	printf(
		"\tpose -> HDING(%f, %f) LOC(%f, %f)\n",
		snap->pose.heading.x, snap->pose.heading.y,
		snap->pose.pos.x, snap->pose.pos.y
	);
}
//---------------------------------------------------------------------------
sysSHM_t* sysAttachSHM()
{
	int shmid = 0;
	key_t key = 0xBEEFCAFE;

	if ((shmid = shmget(key, sizeof(sysSHM_t), IPC_CREAT | 0666)) < 0) {
		perror("shmget");
		return NULL;
	}

	sysSHM_t* mem = shmat(shmid, NULL, 0);
	if(mem == (void*)-1){
		return NULL;
	}

	return mem;
}
