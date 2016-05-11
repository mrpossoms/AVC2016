#include "agents.h"

static void routeInit(void)
{
	// Add any adj states here, do other setup
}
//------------------------------------------------------------------------------
static float utility(agent_t* current, void* args)
{
	// if no route is loaded, there's no where to go
	// thus nothing to steer toward
	if(!SYS.route.start || !SYS.route.currentWaypoint){
		return 0;
	}

	gpsWaypointCont_t* waypoint = SYS.route.currentWaypoint;
	vec3f_t delta = vec3fSub(&SYS.body.measured.position, &waypoint->self.location);

	return 10 / vec3fMag(&delta);
}
//------------------------------------------------------------------------------
static void* action(agent_t* lastState, void* args)
{
	static float last_dist;

	if(!SYS.route.start || !SYS.route.currentWaypoint){
		return NULL;
	}

	if(SYS.following){
		if(SYS.shm->updatedTime > time(NULL) - 5)
		{
			SYS.route.currentWaypoint = (gpsWaypointCont_t*)(&SYS.shm->followLocation);
		}
	}
	else{
		gpsWaypointCont_t* waypoint = SYS.route.currentWaypoint;
		vec3f_t delta = vec3fSub(&SYS.body.measured.position, &waypoint->self.location);
		delta.z = 0; // we don't give a shit about altitude

		float dist = vec3fMag(&delta);
		float d_dist = last_dist - dist; // distance delta

		SYS.body.estimated.goalHeading = vec3fNorm(&delta);

		// less than 6 meters away, lets move on to the next one
		if(d_dist < 0 && dist < 8){
			waypoint->self.flags++;
			SYS.route.currentWaypoint = waypoint->next;
		}
	}
	

	// do stuff here, choose a successor state if appropriate
	return NULL;
}
//------------------------------------------------------------------------------
static void stimulate(float weight, agent_t* stimulator)
{
	// when other agents call this function, they increase
	// the value returned by the utility function
}

agent_t AGENT_ROUTING = {
	utility,      // utility function
	0,            // utility value (will be discarded)
	NULL,         // adjacent state array
	0,            // adjacent state count
	routeInit, // initializer function
	action,       // action function
	stimulate,    // stimulation function
};
