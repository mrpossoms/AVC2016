#include "gps.h"

#include <math.h>
#include <unistd.h>
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

	vec3f_t heading  = { position->x - lastPos.x, position->y - lastPos.y, position->z - lastPosition.z };
	float headingMag = sqrt(heading.x * heading.x + heading.y * heading.y + heading.z * heading.z);

	velocity->x = GPS_STATE.Speed * heading.x / headingMag;
	velocity->y = GPS_STATE.Speed * heading.y / headingMag;
	velocity->z = GPS_STATE.Speed * heading.z / headingMag;

	LAST_CHK_SUM = GPS_STATE.checksum;

	return GPS_STATE.Fix;
}
