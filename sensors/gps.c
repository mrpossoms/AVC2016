#include "gps.h"

#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include <libNEMA.h>

static gpsState_t GPS_STATE;
static unsigned char LAST_CHK_SUM;
static pthread_t GPS_THREAD;

static void* gpsWorker(void* args)
{
	char buf[256] = {};


	while(1){
		lnReadMsg(buf, sizeof(buf));
		lnParseMsg(&GPS_STATE, buf);
		usleep(10000);
	}

	return NULL;
}
//-----------------------------------------------------------------------------
int gpsInit(const char* devPath)
{
	if(lnConnect(devPath, 9600) <= 0){
		return -1;
	}

	pthread_create(&GPS_THREAD, NULL, gpsWorker, NULL);

	return 0;
}
//-----------------------------------------------------------------------------
int gpsHasNewReadings()
{
	return LAST_CHK_SUM != GPS_STATE.checksum;
}
//-----------------------------------------------------------------------------
int gpsGetReadings(vec3f_t* position, vec3f_t* velocity)
{
	const float dia = 6371000 * 2;

	float latRad = GPS_STATE.Lat * (M_PI / 180.0f);	
	float lonRad = GPS_STATE.Lon * (M_PI / 180.0f);	

	// circumferance = d * pi

	vec3f_t lastPos = *position;	

	position->x = dia * lonRad;
	position->y = dia * latRad;
	position->z = GPS_STATE.Altitude;

	vec3f_t heading  = { position->x - lastPos.x, position->y - lastPos.y, position->z - lastPos.z };
	float headingMag = sqrt(heading.x * heading.x + heading.y * heading.y + heading.z * heading.z);

	velocity->x = GPS_STATE.Speed * heading.x / headingMag;
	velocity->y = GPS_STATE.Speed * heading.y / headingMag;
	velocity->z = GPS_STATE.Speed * heading.z / headingMag;

	LAST_CHK_SUM = GPS_STATE.checksum;

	return GPS_STATE.Fix;
}
//-----------------------------------------------------------------------------
float gpsDistToWaypoint(vec3f_t* position, gpsWaypointCont_t* waypoint)
{
	float dx = position->x - waypoint->self.location.x;
	float dy = position->y - waypoint->self.location.y;
	return sqrt(dx * dx + dy * dy);
}
//-----------------------------------------------------------------------------
float gpsDistToWaypoint3D(vec3f_t* position, gpsWaypointCont_t* waypoint)
{
	vec3f_t h = gpsHeadingToWaypoint(position, waypoint);

	return sqrt(h.x * h.x + h.y * h.y + h.z * h.z);
}
//-----------------------------------------------------------------------------
vec3f_t gpsHeadingToWaypoint(vec3f_t* position, gpsWaypointCont_t* waypoint)
{
	vec3f_t heading = {
		position->x - waypoint->self.location.x,
		position->y - waypoint->self.location.y,
		position->z - waypoint->self.location.z,
	};

	return heading;
}
//-----------------------------------------------------------------------------
int gpsRouteLoad(const char* path, gpsWaypointCont_t** waypoints)
{
	int fd = open(path, O_RDONLY);

	if(fd <= 0){
		return -1;
	}

	gpsRouteHeader_t header = {};

	if(read(fd, &header, sizeof(header)) != sizeof(header)){
		return -2;
	}

	if(!(*waypoints = (gpsWaypointCont_t*)malloc(sizeof(gpsWaypointCont_t) * header.waypoints))){
		return -3;
	}

	gpsWaypointCont_t* last = NULL;
	for(int i = 0; i < header.waypoints; ++i){
		if(read(fd, (*waypoints) + i, sizeof(gpsWaypoint_t)) != sizeof(gpsWaypoint_t)){
			free(*waypoints);
			close(fd);
			return -4;
		}

		(*waypoints)[i].next = NULL;

		if(last){
			last->next = (*waypoints) + i;
		}

		last = (*waypoints) + i;
	}

	close(fd);

	return 0;
}
//-----------------------------------------------------------------------------
int gpsRouteAdvance(vec3f_t* position, gpsWaypointCont_t** current, uint8_t lapFlag)
{
	if(!position) return -1;
	if(!current) return -2;

	gpsDistToWaypoint* waypoint = *current;
	if(gpsDistToWaypoint(position, waypoint) <= waypoint->self.tolerance){
		*current = waypoint->next;
		return 1;
	}

	return 0;
}
