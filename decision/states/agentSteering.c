#include "decision/agents.h"
#include "controls/servos.h"
#include "system.h"

#include <string.h>

static void steeringInit(void)
{
	// Add any adj states here, do other setup
}

static float angleToNextWayPoint(objectState_t* o, gpsWaypointCont_t* waypoint)
{
	const vec3f_t zero = {};
	if(!memcmp(&o->position, &zero, sizeof(vec3f_t))) return 0;
 
	vec3f_t toWaypoint = vec3fSub(&o->position, &waypoint->self.location);
	return vec3fAng(&o->heading, &toWaypoint);
}

static float utility(agent_t* current, void* args)
{
	// if no route is loaded, there's no where to go
	// thus nothing to steer toward
	if(!SYS.route.start || !SYS.route.currentWaypoint){
		return 0;
	}

	// the bigger the angle the more important the correction
	return fabs(angleToNextWayPoint(&SYS.body.measured, SYS.route.currentWaypoint));
}

static void* action(agent_t* lastState, void* args)
{
	float ang = angleToNextWayPoint(&SYS.body.measured, SYS.route.currentWaypoint);

	// bound the steering angle
	if(ang >  M_PI / 4){ ang =  M_PI / 4; }
	if(ang < -M_PI / 4){ ang = -M_PI / 4; }

	// map to range [-1, 1]
	ang /= (M_PI / 4);

	// steer within the range [25%, 75%]
	conSet(SERVO_STEERING, ang * 25 + 50);

	return NULL;
}

static void stimulate(float weight, agent_t* stimulator)
{
	// when other agents call this function, they increase
	// the value returned by the utility function
}

agent_t AGENT_STEERING = {
	utility,      // utility function
	0,            // utility value (will be discarded)
	NULL,         // adjacent state array
	0,            // adjacent state count
	steeringInit, // initializer function
	action,       // action function
	stimulate,    // stimulation function
};
