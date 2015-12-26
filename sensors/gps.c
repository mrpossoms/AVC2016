#include "gps.h"

#include <libNEMA.h>
#include <unistd.h>
#include <pthread.h>

static gpsState_t GPS_STATE;
static unsigned char LAST_CHK_SUM;

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

	return 0;
}
//-----------------------------------------------------------------------------
int gpsHasNewReadings()
{
	return LAST_CHK_SUM != GPS_STATE.checksum;
}
//-----------------------------------------------------------------------------
int gpsGetReadings(vec3f_t* position, vec3f_t* veclocity)
{
	position->x = GPS_STATE.Lon;
	position->y = GPS_STATE.Lat;
	position->z = GPS_STATE.Altitude;

	veclocity->x = GPS_STATE.Speed;

	LAST_CHK_SUM = GPS_STATE.checksum;

	return 0;
}