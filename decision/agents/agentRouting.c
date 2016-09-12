#include "agents.h"
#include "sensors/gps.h"

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
static float waypoint_cost(gpsWaypointCont_t* way)
{
	const float min_dist = 0.00001;

	vec3f_t delta = {};
	delta.x = SYS.pose.pos.x - way->self.location.x;
	delta.y = SYS.pose.pos.y - way->self.location.y;
	delta.z = 0; // we don't give a shit about altitude

	float dist = vec3fMag(&delta);
	float coincidence = 0;
	vec3f_t grad = gpsWaypointGradient(way);
	vec3f_t* heading = &SYS.pose.heading;

	if(grad.x || grad.y || grad.z)
	{
		float base = vec3fMag(&grad) * vec3fMag(heading);
		coincidence = acosf(vec3fDot(&grad, heading) / base);
	}

	return powf(dist - min_dist, 2) + coincidence;
}
//------------------------------------------------------------------------------
static void standard_routing()
{
	static float last_dist;
	static float last_d_dist;

	gpsWaypointCont_t* waypoint = SYS.route.currentWaypoint;
	vec3f_t delta = {};
	delta.x = SYS.pose.pos.x - waypoint->self.location.x;
	delta.y = SYS.pose.pos.y - waypoint->self.location.y;
	delta.z = 0; // we don't give a shit about altitude

	// SYS.body.estimated.heading.goal = vec3fNorm(&delta);
	float dist = vec3fMag(&delta);
	float d_dist = last_dist - dist;

	// less than 6 meters away, lets move on to the next one
	if((last_d_dist <= 0 && d_dist >= 0) && dist < 0.00001){
		waypoint->self.flags++;
		SYS.route.currentWaypoint = waypoint->next;
	}

	last_dist = dist;
	last_d_dist = d_dist;
}
//------------------------------------------------------------------------------
static void cost_routing()
{
	gpsWaypointCont_t* waypoint = SYS.route.currentWaypoint;

	vec3f_t* h = &SYS.pose.heading;
	
	if(vec3fIsNan(h)) return;
	if(!h->x && !h->y && !h->z)
	{
		return;
	}

	// are we near the end of our route?
	if(!waypoint->next)
	{
		// just check to see if we get close to the final waypoint
		// then. There is no need to try to select one
		vec3f_t delta = {};
		delta.x = SYS.pose.pos.x - waypoint->self.location.x;
		delta.y = SYS.pose.pos.y - waypoint->self.location.y;
		delta.z = 0; // we don't give a shit about altitude

		if(vec3fMag(&delta) < 0.00001){
			waypoint->self.flags++;
			SYS.route.currentWaypoint = waypoint->next;
		}

		return;
	}

	// start with our current waypoint
	float best_cost = waypoint_cost(waypoint);
	gpsWaypointCont_t* best_way = waypoint;

	// iterate over all remaining waypoints
	for(gpsWaypointCont_t* way = best_way->next; way; way = way->next)
	{
		// is there a more optimal one to seek?
		float cost = waypoint_cost(way);
		if(cost < best_cost)
		{
			printf("%x --> %x\n", (unsigned int)best_way, (unsigned int)way);
			best_way = way;
			best_cost = cost;
		}
	}

	if(best_way != waypoint)
	{
		waypoint->self.flags++;
	}

	SYS.route.currentWaypoint = best_way;
}
//------------------------------------------------------------------------------
static void* action(agent_t* lastState, void* args)
{
	if(!SYS.route.start || !SYS.route.currentWaypoint){
		return NULL;
	}

	if(SYS.following && SYS.shm){
		if(SYS.shm->updatedTime > time(NULL) - 5)
		{
			SYS.route.currentWaypoint = (gpsWaypointCont_t*)(&SYS.shm->followLocation);
		}
	}
	else{
		//standard_routing();
		cost_routing();
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
