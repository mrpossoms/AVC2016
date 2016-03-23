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
	vec3f_t acc = SYS.body.imu.adjReadings.linear;
	acc.x = 0;

	// do stuff here, choose a successor state if appropriate
	if(vec3fMag(&acc) > LIL_G + 5){
		// set the current waypoint to NULL, this will terminate the
		// program
		printf("IMPACT DETECTED\n");
		SYS.route.currentWaypoint = NULL;
	}

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
