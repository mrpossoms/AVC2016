#include "agents.h"

static void crashInit(void)
{
	// Add any adj states here, do other setup
}

static float utility(agent_t* current, void* args)
{
	// return the measure of utility given by this function
	// when considering the args, and the state of the system.
	return 0;
}

static int impact_danger()
{
	// scn_obstacle_t* obs = SYS.sensors.scanner.obstacles;
	// vec3f_t v0 = {};
	// vec3f_t v1 = {};
	// vec3f_t intersect_point = {};
	//
	// v0.x = SYS.pose.pos.x; v0.y = SYS.pose.pos.y; v0.z = SYS.pose.pos.z;
	//
	// vec3Scl(v1, SYS.pose.vel, mtodeg(1) * -2);
	// vec3Add(v1, v1, v0);
	//
	// for(int i = 0; i < SCANNER_RES; ++i)
	// {
	// 	if(!obs[i].valid) continue;
	//
	// 	if(obs_intersect(obs + i, v0, v1, &intersect_point) == 1)
	// 	{
	//
	// 		printf("obs%d dist:%f rad:%f width:%f\n",
	// 			i,
	// 			obs[i].nearest,
	// 			obs[i].radius,
	// 			obs[i].width);
	// 		return 1;
	// 	}
	// }

	scn_obstacle_t* obs;
	gpsWaypointCont_t* before_intersect;
	if((obs = obs_intersects_route(
		SYS.sensors.scanner.obstacles,
		SCANNER_RES,
		SYS.route.currentWaypoint,
		&before_intersect)))
	{
		obs_print_info(obs);
		return 1;
	}

	return 0;
}

static void* action(agent_t* lastState, void* args)
{
	static float last_mag;
	vec3f_t acc = SYS.sensors.measured.acc;
	acc.x = 0;
	acc.z = 0; // don't catch jumps and stuff
	float mag = vec3fMag(&acc);

	if(last_mag == 0)
	{
		last_mag = mag;
	}

	int has_impacted = fabs(last_mag - mag) > LIL_G;

	// do stuff here, choose a successor state if appropriate
	if(has_impacted || impact_danger()){
		// set the current waypoint to NULL, this will terminate the
		// program
		if(has_impacted) printf("IMPACT DETECTED\n");
		if(!has_impacted) printf("CLOSE OBSTACLE\n");
		SYS.route.currentWaypoint = NULL;
	}

	last_mag = mag;

	return NULL;
}

static void stimulate(float weight, agent_t* stimulator)
{
	// when other agents call this function, they increase
	// the value returned by the utility function
}

agent_t AGENT_CRASH_DETECTOR = {
	utility,      // utility function
	0,            // utility value (will be discarded)
	NULL,         // adjacent state array
	0,            // adjacent state count
	crashInit, // initializer function
	action,       // action function
	stimulate,    // stimulation function
};
