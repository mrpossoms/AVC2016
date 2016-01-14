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
	body->measured.heading.x = body->imu.adjReadings.mag.x;
	body->measured.heading.y = body->imu.adjReadings.mag.y;
	body->measured.heading.z = body->imu.adjReadings.mag.z;
	body->measured.heading = vec3fNorm(&body->measured.heading);

	if(gpsHasNewReadings()){
		float dt = SYS.timeUp - body->lastMeasureTime;
		vec3f_t lastPos = body->measured.position;
		

		// assign new measurements
		gpsGetReadings(&body->measured.position, &body->measured.velocity);
		vec3Sub(body->measured.velocity, body->measured.position, lastPos); 
		vec3Scl(body->measured.velocity, body->measured.velocity, dt);

		// since we now have measurements, reset the estimates
		body->estimated = body->measured;

		body->lastMeasureTime = SYS.timeUp;
		body->lastEstTime     = SYS.timeUp;
	}
	else
	{
		float dt = SYS.timeUp - body->lastEstTime;

		// integrate position using velocity
		body->estimated.position.x += body->estimated.velocity.x * dt; 
		body->estimated.position.y += body->estimated.velocity.y * dt;
		body->estimated.position.z += body->estimated.velocity.z * dt;

		// integrate IMU acceleration into velocity 
		body->estimated.velocity.x += body->imu.adjReadings.linear.x * dt; 
		body->estimated.velocity.y += body->imu.adjReadings.linear.y * dt;
		body->estimated.velocity.z += body->imu.adjReadings.linear.z * dt;

		body->lastEstTime = SYS.timeUp;
	}
	return 0;
}
