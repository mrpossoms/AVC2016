#include "decision/agents.h"
#include "controls/servos.h"
#include "system.h"

static void throttleInit(void)
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
	// do stuff here, choose a successor state if appropriate
	if(SYS.route.currentWaypoint && SYS.body.hasGpsFix){
		ctrlSet(SERVO_THROTTLE, 53);
	}
	else{
		ctrlSet(SERVO_THROTTLE, 50);
	}
	return NULL;
}

static void stimulate(float weight, agent_t* stimulator)
{
	// when other agents call this function, they increase
	// the value returned by the utility function
}

agent_t AGENT_THROTTLE = {
	utility,      // utility function
	0,            // utility value (will be discarded)
	NULL,         // adjacent state array
	0,            // adjacent state count
	throttleInit, // initializer function
	action,       // action function
	stimulate,    // stimulation function
};
