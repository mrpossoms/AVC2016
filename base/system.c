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
	snap.imu.adj = sys->body.imu.filtered;

	gpsWaypointCont_t* waypoint = sys->route.currentWaypoint;
	if(waypoint){
		snap.currentWaypoint = waypoint->self;

		if(waypoint->next){
			snap.nextWaypoint = waypoint->next->self;
		}
	}

	return snap;
}
