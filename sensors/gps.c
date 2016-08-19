#include "gps.h"

#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <assert.h>

#define GPS_HZ 5

gpsState_t GPS_STATE;
static unsigned char LAST_CHK_SUM;
static pthread_t GPS_THREAD;
static int UPDATES;
static vec3d_t SAMPLES[GPS_HZ];

static void* gpsWorker(void* args)
{
	char buf[512] = {};

	while(1){
		lnReadMsg(buf, sizeof(buf));
		if(lnParseMsg(&GPS_STATE, buf) == LN_GPGLL){
			// fresh coordinates
			int i = UPDATES % GPS_HZ;

			++UPDATES;
			i = UPDATES % GPS_HZ;
			SAMPLES[i].x = GPS_STATE.Lon;
			SAMPLES[i].y = GPS_STATE.Lat;
			SAMPLES[i].z = GPS_STATE.Altitude;

			LAST_CHK_SUM = GPS_STATE.checksum;
		}
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
	//if(LAST_CHK_SUM != GPS_STATE.checksum || !LAST_CHK_SUM){


	//}

	return (UPDATES % GPS_HZ) == 0;
}
//-----------------------------------------------------------------------------
vec3d_t mtoll(vec3d_t* meters)
{
	const double dia = 6371000 * 2;
	double latRad = coord->y * (M_PI / 180.0f);
	double lonRad = coord->x * (M_PI / 180.0f);

	vec3d_t ll = {
		meters->x * 180 / (dia * M_PI),
		meters->y * 180 / (dia * M_PI),
		meters->z
	};

	return ll;
}
//-----------------------------------------------------------------------------
int gpsGetReadings(vec3d_t* position, vec3f_t* heading)
{
	position->x = position->y = position->z = 0;

	for(int i = GPS_HZ; i--;){
		position->x += SAMPLES[i].x / (double)GPS_HZ;
		position->y += SAMPLES[i].y / (double)GPS_HZ;
		position->z += SAMPLES[i].z / (double)GPS_HZ;
	}

	// mi casa
	if(!position->x && !position->y){
		position->x = -85.651659;
		position->y = 42.962689;
	}

	UPDATES += 1;

	return GPS_STATE.Fix;
}
//-----------------------------------------------------------------------------
float gpsDistToWaypoint(vec3d_t* position, gpsWaypointCont_t* waypoint)
{
	float dx = position->x - waypoint->self.location.x;
	float dy = position->y - waypoint->self.location.y;
	return sqrt(dx * dx + dy * dy);
}
//-----------------------------------------------------------------------------
float gpsDistToWaypoint3D(vec3d_t* position, gpsWaypointCont_t* waypoint)
{
	vec3f_t h = gpsHeadingToWaypoint(position, waypoint);

	return sqrt(h.x * h.x + h.y * h.y + h.z * h.z);
}
//-----------------------------------------------------------------------------
vec3f_t gpsHeadingToWaypoint(vec3d_t* position, gpsWaypointCont_t* waypoint)
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

	if(!(*waypoints = (gpsWaypointCont_t*)calloc(header.waypoints, sizeof(gpsWaypointCont_t)))){
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

		vec3d_t loc = (*waypoints)[i].self.location;
		(*waypoints)[i].next = NULL;

		if(last){
			last->next = (*waypoints) + i;
		}

		last = (*waypoints) + i;
	}

	assert(last->next == NULL);
	close(fd);

	return 0;
}
//-----------------------------------------------------------------------------
int gpsRouteAdvance(vec3d_t* position, gpsWaypointCont_t** current, uint8_t lapFlag)
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
