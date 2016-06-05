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

	return 10 / vec3Dist(SYS.pose.pos, waypoint->self.location);
}
//------------------------------------------------------------------------------
static void* action(agent_t* lastState, void* args)
{
	static float last_dist;
	static float last_d_dist;

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
		vec3f_t delta = {};
		delta.x = SYS.pose.pos.x - waypoint->self.location.x;
		delta.y = SYS.pose.pos.y - waypoint->self.location.y;
		delta.z = 0; // we don't give a shit about altitude

		// SYS.body.estimated.heading.goal = vec3fNorm(&delta);
		float dist = vec3fMag(&delta);
		float d_dist = last_dist - dist;

		// less than 6 meters away, lets move on to the next one
		if((last_d_dist <= 0 && d_dist >= 0) && dist < 6){
			waypoint->self.flags++;
			SYS.route.currentWaypoint = waypoint->next;
		}

		last_dist = dist;
		last_d_dist = d_dist;
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
