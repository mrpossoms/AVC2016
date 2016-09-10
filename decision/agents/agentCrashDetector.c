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

static void* action(agent_t* lastState, void* args)
{
	static float last_mag;
	vec3f_t acc = SYS.sensors.measured.acc;
	acc.x = 0;
	float mag = vec3fMag(&acc);

	scn_obstacle_t* nearest = SYS.sensors.scanner.obstacles;

	if(last_mag == 0)
	{
		last_mag = mag;
	}

	int has_impacted = fabs(last_mag - mag) > LIL_G / 2;

	float dist_to_obs = vec3Dist(nearest->centroid, SYS.pose.pos);
	float speed = vec3_len(SYS.pose.vel.v);
	int danger_of_impact = dist_to_obs < speed; // travel vector for next 2 seconds

	// do stuff here, choose a successor state if appropriate
	if(has_impacted || danger_of_impact){
		// set the current waypoint to NULL, this will terminate the
		// program
		if(has_impacted) printf("IMPACT DETECTED\n");
		if(danger_of_impact) printf("CLOSE OBSTACLE %fM @ %fM/s\n", dist_to_obs, speed);
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
