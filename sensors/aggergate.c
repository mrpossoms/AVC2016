#include "aggergate.h"

#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <libNEMA.h>

static int FD_IMU;

//-----------------------------------------------------------------------------
static int openSensor(const char* dev, int* fd, int flags)
{
	*fd = open(dev, flags);
	return fd > 0;
}
//-----------------------------------------------------------------------------
int senInit(const char* imuDevice, const char* gpsDevice, const char* calProfile)
{
	printf("Initializing GPS...");
	if(gpsInit(gpsDevice)){
		printf("Failed!\n");
		return -1;
	}
	printf("OK!\n");

	printf("Initializing IMU...");
	if(!openSensor(imuDevice, &FD_IMU, O_RDWR)){
		printf("Failed!\n");
		return -2;
	}
	printf("OK!\n");

	if(calProfile){
		int calFd = open(calProfile, O_RDONLY);

		if(calFd <= 0 || imuLoadCalibrationProfile(calFd, &SYS.body.imu)){
			close(calFd);
			return -3;
		}

		close(calFd);
	}

	return 0;
}
//-----------------------------------------------------------------------------
int senShutdown()
{
	if(gpsShutdown()) return -1;
	if(close(FD_IMU)) return -2;

	return 0;
}
//-----------------------------------------------------------------------------
static vec3f_t senMesHeading(sensorStatef_t* r)
{
	// update the heading according to magnetometer readings
	vec3f_t heading = {
		-r->mag.x,
		-r->mag.y,
		 r->mag.z
	};
	return vec3fNorm(&heading);
}
//-----------------------------------------------------------------------------
int senUpdate(fusedObjState_t* body)
{
	imuUpdateState(FD_IMU, &body->imu);

	// update the heading according to magnetometer readings
	body->measured.heading = senMesHeading(&body->imu.adjReadings);

	if(gpsHasNewReadings()){
		float dt = SYS.timeUp - body->lastMeasureTime;
		vec3f_t lastPos = body->measured.position;


		// assign new ements
		vec3f_t* velLin = &body->measured.velocity.linear;
		body->hasGpsFix = gpsGetReadings(&body->measured.position, velLin);
		vec3Sub(*velLin, body->measured.position, lastPos);
		vec3Scl(*velLin, *velLin, dt);

		// since we now have measurements, reset the estimates
		body->estimated = body->measured;

		body->lastMeasureTime = SYS.timeUp;
		body->lastEstTime     = SYS.timeUp;
	}
	else
	{
		float dt = SYS.timeUp - body->lastEstTime;

		// integrate position using velocity
		vec3f_t* estVelLin = &body->estimated.velocity.linear;
		body->estimated.position.x += estVelLin->x * dt;
		body->estimated.position.y += estVelLin->y * dt;
		body->estimated.position.z += estVelLin->z * dt;

		// integrate IMU acceleration into velocity
		estVelLin->x += body->imu.adjReadings.linear.x * dt;
		estVelLin->y += body->imu.adjReadings.linear.y * dt;
		estVelLin->z += body->imu.adjReadings.linear.z * dt;

		body->lastEstTime = SYS.timeUp;
	}
	return 0;
}
