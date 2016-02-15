#include "decision/agents.h"
#include "system.h"

static void templateInit(void)
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
	return NULL;
}

static void stimulate(float weight, agent_t* stimulator)
{
	// when other agents call this function, they increase
	// the value returned by the utility function
}

agent_t AGENT_TEMPLATE = {
	utility,      // utility function
	0,            // utility value (will be discarded)
	NULL,         // adjacent state array
	0,            // adjacent state count
	templateInit, // initializer function
	action,       // action function
	stimulate,    // stimulation function
};
