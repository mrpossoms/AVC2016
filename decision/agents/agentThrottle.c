#include "agents.h"
#include "controls/servos.h"

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
	gpsWaypointCont_t* waypoints[2] = {};
	int waypointCount = 0;
	//const float speed = vec3fMag(&SYS.body.measured.velocity.linear);
	float throttle = SYS.maxSpeed;
	vec3f_t d1 = vec3fSub(&SYS.body.measured.position, &waypoints[0]->self.location);

	for(gpsWaypointCont_t* point = SYS.route.currentWaypoint; point && waypointCount < 2; waypointCount++){
		waypoints[waypointCount] = point;
		point = point->next;
	}

	// if more than one non null waypoints are accounted for
	if(waypointCount > 1){
		// compute deltas from position to W0 and from W0 to W1
		vec3f_t delta[2] = {
			d1,
			vec3fSub(&waypoints[0]->self.location, &waypoints[1]->self.location)
		};
		
		// normalize both delta vectors
		delta[0] = vec3fNorm(delta + 0);
		delta[1] = vec3fNorm(delta + 1);

		// now the dot between both deltas will lie in the range [-1,1]
		// add one to bring it to [0, 2]
		float AoT = vec3fDot(delta + 0, delta + 1) + 1;
		AoT = (AoT / 2)	+ (1.0f / 3.0f); // bring AoT to [0, 1] bias with 1/3

		// scale the difference between maxSpeed and stopped (50) by AoT
		// if the result is less than 52, then set it to 52 explicitly
		throttle = 50 + ((SYS.maxSpeed - 50) * AoT);
		float adjThrottle = throttle < 52 ? 52 : throttle;
		float dist = vec3fMag(&d1);
		float p = dist / 10;
		
		p = p > 1 ? 1 : p;
		throttle = throttle * p + adjThrottle * (1 - p);
	}

	// do stuff here, choose a successor state if appropriate
	if(SYS.route.currentWaypoint && SYS.body.hasGpsFix && vec3fMag(&d1) > 8){
		ctrlSet(SERVO_THROTTLE, throttle);
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
