#include "agents.h"
#include "sensors/gps.h"
#include <assert.h>

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
static void standard_routing()
{
	static float last_dist;
	static float last_d_dist;
	float min_dist = mtodeg(1);

	gpsWaypointCont_t* waypoint = SYS.route.currentWaypoint;
	vec3f_t delta = {};
	delta.x = SYS.pose.pos.x - waypoint->self.location.x;
	delta.y = SYS.pose.pos.y - waypoint->self.location.y;
	delta.z = 0; // we don't give a shit about altitude

	if(!SYS.route.currentWaypoint->next)
	{
		min_dist *= 2;
	}

	// SYS.body.estimated.heading.goal = vec3fNorm(&delta);
	float dist = vec3fMag(&delta);
	float d_dist = last_dist - dist;

	// less than 6 meters away, lets move on to the next one
	if((last_d_dist <= 0 && d_dist >= 0) && dist < min_dist){
		SYS.route.currentWaypoint = waypoint->next;
		printf("FINISED\n");
	}

	last_dist = dist;
	last_d_dist = d_dist;
}
//------------------------------------------------------------------------------
static void cost_routing()
{
	gpsWaypointCont_t* waypoint = SYS.route.currentWaypoint;
	const float min_dist = mtodeg(0.1);

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

		if(vec3fMag(&delta) < min_dist){
			SYS.route.currentWaypoint = waypoint->next;

			printf("Finished!\n");
		}

		return;
	}

	// start with our current waypoint
	float best_cost = waypoint_cost(waypoint, &SYS.pose);
	gpsWaypointCont_t* best_way = waypoint;

	// iterate over all remaining waypoints
	for(gpsWaypointCont_t* way = best_way->next; way; way = way->next)
	{
		// is there a more optimal one to seek?
		float cost = waypoint_cost(way, &SYS.pose);
		if(cost < best_cost)
		{
			best_cost = cost;
			printf("%x --> %x %f\n",
				(unsigned int)best_way,
				(unsigned int)way,
				best_cost
			);
			best_way = way;
			if(!best_way->next)
			{
				printf("LAST\n");
			}
		}
	}

	SYS.route.currentWaypoint = best_way;
}
//------------------------------------------------------------------------------
static int reroute(scn_t* scn, scn_obstacle_t* obs, gpsWaypointCont_t* before)
{
	const float car_width = mtodeg(0.4); // 40cm
	float safe_rad = (obs->radius/4 + car_width);
	
	vec3f_t delta = {};
	vec3Sub(
		delta,
		scn->readings[obs->right_i-1].location,
		scn->readings[obs->left_i+1].location
	);

	// start from the current waypoint, walk through the obstacles
	gpsWaypointCont_t* way = SYS.route.start;
	for(; way; way = way->next)
	{
		float dist = vec3Dist(obs->centroid, way->self.location);
		
		assert(dist > 0);

		if(dist <= safe_rad)
		{
			vec3f_t n = {};
			printf("%d(%f) - %d(%f)\n",
			obs->left_i, scn->readings[obs->left_i+1].distance,
			obs->right_i, scn->readings[obs->right_i-1].distance);

			//delta.z = 0.0001;

			// normalize
			//vec3Scl(n, delta, 1 / dist);
			vec3Scl(n, delta, .01);

			// offset
			vec3Add(way->self.location, way->self.location, n);	
		}

		//break;
	}

	return 1;
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
		standard_routing();
		//cost_routing();


		// while the route is intersected by obstacles
		// keep rerouting
		scn_obstacle_t* obs = NULL;
		gpsWaypointCont_t* before_intersect = NULL;

		if(SYS.use_scanner)
		{
			obs = obs_intersects_route(
				SYS.sensors.scanner.obstacles,
				SCANNER_RES,
				SYS.route.currentWaypoint,
				&before_intersect
			);

			if(obs) // an intersection wasn't detected, we're good
			{
				ctrlSet(SERVO_THROTTLE, 50);
				reroute(&SYS.sensors.scanner, obs, before_intersect);
			}
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
