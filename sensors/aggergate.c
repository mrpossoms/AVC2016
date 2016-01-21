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
int senUpdate(fusedObjState_t* body)
{
	imuUpdateState(FD_IMU, &body->imu);

	// update the heading according to magnetometer readings
	body->measured.heading.x = -body->imu.adjReadings.mag.x;
	body->measured.heading.y = -body->imu.adjReadings.mag.y;
	body->measured.heading.z =  body->imu.adjReadings.mag.z;
	body->measured.heading = vec3fNorm(&body->measured.heading);

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
