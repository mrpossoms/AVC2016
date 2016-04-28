#include <sys/shm.h>
#include "system.h"

//     ___ _     _          _
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//
system_t SYS;

sysSnap_t sysSnapshot(system_t* sys)
{
	sysSnap_t snap = {};

	snap.estimated = sys->body.estimated;
	snap.hasGpsFix = sys->body.hasGpsFix;
	snap.imu.raw = sys->body.imu.raw;
	snap.imu.cal = sys->body.imu.cal;
	snap.imu.filtered = sys->body.imu.filtered;

	gpsWaypointCont_t* waypoint = sys->route.currentWaypoint;
	if(waypoint){
		snap.currentWaypoint = waypoint->self;

		if(!sys->following && waypoint->next){
			snap.nextWaypoint = waypoint->next->self;
		}
	}

	return snap;
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
