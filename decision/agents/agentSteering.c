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
	vec3f_t tempHeading = o->heading;	
	toWaypoint.x *= -1;

	toWaypoint.z = tempHeading.z= 0; 
	toWaypoint = vec3fNorm(&toWaypoint);
	tempHeading = vec3fNorm(&tempHeading);

	float d1 = vec3fDot(&toWaypoint, &tempHeading);
	
	float x = cos(M_PI / 2) * tempHeading.x - sin(M_PI / 2) * tempHeading.y;
	float y = sin(M_PI / 2) * tempHeading.x + cos(M_PI / 2) * tempHeading.y;
	tempHeading.x = x; tempHeading.y = y;

	float d2 = vec3fDot(&toWaypoint, &tempHeading);

	// when the heading is parallel with the goal heading, but pointing
	// in the opposite direction d1 will approach -1, and d2 will approach 0
	// clamping these values will prevent the vehicle from steering straight when
	// it is facing the opposite direction of the goal
	if(d1 < -0.95f){
		d1 = -0.95;
		d2 = 0.22;
	}

	if(SYS.debugging){
		printf("toWay . heading = %f\n", vec3fDot(&toWaypoint, &o->heading));
		printf(
			"(%f, %f) -> (%f, %f)\n",
			o->position.x, o->position.y,
			waypoint->self.location.x, waypoint->self.location.y
		);

		printf("heading %f, %f\n", o->heading.x, o->heading.y);
		printf("delta %f, %f\n", toWaypoint.x, toWaypoint.y);
		printf("d1 = %f, d2 = %f\n", d1, d2);
		usleep(1000 * 250);
	}

	return acos(d1) * d2;
}

static float utility(agent_t* current, void* args)
{
	// if no route is loaded, there's no where to go
	// thus nothing to steer toward
	if(!SYS.route.start || !SYS.route.currentWaypoint){
		return 0;
	}

	// the bigger the angle the more important the correction
	return fabs(angleToNextWayPoint(&SYS.body.estimated, SYS.route.currentWaypoint));
}

static void* action(agent_t* lastState, void* args)
{
	if(!SYS.route.start || !SYS.route.currentWaypoint){
		return NULL;
	}

	float ang = angleToNextWayPoint(&SYS.body.estimated, SYS.route.currentWaypoint);

	if(SYS.debugging){
		printf("angToNextWaypoint = %f\n", ang);
	}

	// bound the steering angle
	if(ang >  M_PI / 4){ ang =  M_PI / 4; }
	if(ang < -M_PI / 4){ ang = -M_PI / 4; }

	// map to range [-1, 1]
	ang /= (M_PI / 4);

	// steer within the range [25%, 75%]
	ctrlSet(SERVO_STEERING, ang * 25 + 50);

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
