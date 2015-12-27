#include "decision/states.h"
#include "system.h"

void templateInit(void)
{
	// Add any adj states here, do other setup
}

static float utility(decision_t* current, void* args)
{
	// return the measure of utility given by this function
	// when considering the args, and the state of the system.
	return 0;
}

static void* action(decision_t* lastState, void* args)
{
	// do stuff here, choose a successor state if appropriate
	return NULL;
}

decision_t DEC_TEMPLATE = {
	utility, // utility function
	0,       // utility value (will be discarded)
	NULL,    // adjacent state array
	0,       // adjacent state count
	action,  // action function
};

// assign my initializer
STATE_INITIALIZERS[0]();// = templateInit;
