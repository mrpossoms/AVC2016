#include "gps.h"

#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>

gpsState_t GPS_STATE;
static unsigned char LAST_CHK_SUM;
static pthread_t GPS_THREAD;

static void* gpsWorker(void* args)
{
	char buf[512] = {};

	while(1){
		lnReadMsg(buf, sizeof(buf));
		lnParseMsg(&GPS_STATE, buf);
		usleep(10000);
	}

	return NULL;
}
//-----------------------------------------------------------------------------
static int termiosHack(const char* devPath)
{
	int fd;
	unsigned char cbuf[] = {
		0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1,
		0x18, 0x00, 0x00, 0x30, 0x8a, 0x00, 0x00, 0x00, 0x03,
		0x1c, 0x7f, 0x08, 0x04, 0x02, 0x64, 0x00, 0x11, 0x13,
		0x1a, 0x00, 0x12, 0x0f, 0x17, 0x16, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x85, 0x00, 0x00, 0x01, 0x10,
		0x00, 0x00, 0x01, 0x10, 0x00, 0x00
	}; // hacky termios fix

	if((fd = open(devPath, O_RDWR)) <= 0){
		return -1;
	}

	if(tcsetattr(fd, TCSANOW, (struct termios*)cbuf) < 0){
		close(fd);
		return -2;
	}

	close(fd);

	return 0;
}
//-----------------------------------------------------------------------------
int gpsInit(const char* devPath)
{
	termiosHack(devPath);

	if(lnConnect(devPath, 57600) <= 0){
		return -2;
	}

	pthread_create(&GPS_THREAD, NULL, gpsWorker, NULL);

	return 0;
}
//-----------------------------------------------------------------------------
int gpsShutdown()
{
		if(pthread_cancel(GPS_THREAD)){
			return -1;
		}

		if(lnDisconnect()){
			return -2;
		}

		return 0;
}
//-----------------------------------------------------------------------------
int gpsHasNewReadings()
{
	return LAST_CHK_SUM != GPS_STATE.checksum || !LAST_CHK_SUM;
}
//-----------------------------------------------------------------------------
void latLon2meters(vec3f_t* coord)
{
	const float dia = 6371000 * 2;
	float latRad = coord->y * (M_PI / 180.0f);
	float lonRad = coord->x * (M_PI / 180.0f);

	// circumferance = d * pi

	coord->x = dia * lonRad;
	coord->y = dia * latRad;
}
//-----------------------------------------------------------------------------
int gpsGetReadings(vec3f_t* position, vec3f_t* heading)
{
	vec3f_t lastPos = *position;

	position->x = GPS_STATE.Lon;
	position->y = GPS_STATE.Lat;
	position->z = GPS_STATE.Altitude;

	if(!position->x && !position->y){
		position->x = -85.651659;
		position->y = 42.962689;
	}

	latLon2meters(position);

	vec3f_t delta  = { position->x - lastPos.x, position->y - lastPos.y, position->z - lastPos.z };
	float deltaMag = sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);

	// use GPS to try to determine the heading
	if(deltaMag > 0){
		vec3f_t newHeading = {};
		float d;

		vec3Scl(newHeading, delta, 1 / deltaMag);

		if((d = vec3Dot(newHeading, *heading)) <= 0){
			// the direction reverse, just take the new heading
			*heading = newHeading;
		}
		else{ // otherwise prefer new velocities that are near the current
			vec3Lerp(*heading, *heading, newHeading, d / deltaMag);
		}
	}

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

	printf("loading %d waypoints\n", header.waypoints);

	gpsWaypointCont_t* last = NULL;
	for(int i = 0; i < header.waypoints; ++i){
		if(read(fd, (*waypoints) + i, sizeof(gpsWaypoint_t)) != sizeof(gpsWaypoint_t)){
			free(*waypoints);
			close(fd);
			return -4;
		}

		vec3f_t loc = (*waypoints)[i].self.location;
		if(abs(loc.x) <= 90 || abs(loc.y) <= 90){
			printf("\t(%f lon, %f lat) -> ", loc.x, loc.y);
			latLon2meters(&(*waypoints)[i].self.location);

			printf("(%fm, %fm)\n", (*waypoints)[i].self.location.x, (*waypoints)[i].self.location.y);
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

	gpsWaypointCont_t* waypoint = *current;
	if(gpsDistToWaypoint(position, waypoint) <= waypoint->self.tolerance){
		*current = waypoint->next;
		return 1;
	}

	return 0;
}
