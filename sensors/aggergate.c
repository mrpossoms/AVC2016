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
#define ONCE_START {\
	static int once;\
	if(!once){\
	once = 1;\

#define ONCE_END } }

static void estimateHeading(fusedObjState_t* body, float dt)
{
	objectState_t *mea= &body->measured;
	objectState_t *est= &body->estimated;

	// use the gyro to estimate how confident we should be in the magnetometer's
	// current measured heading
	float w = body->imu.adjReadings.rotational.z / -640000.0f;

	// update the heading according to magnetometer readings
	mea->heading.x = -body->imu.adjReadings.mag.x;
	mea->heading.y = -body->imu.adjReadings.mag.y;
	mea->heading.z =  body->imu.adjReadings.mag.z;
	mea->heading = vec3fNorm(&mea->heading);

	if(vec3fIsNan(&mea->heading)) return;

ONCE_START
	printf("ONCE\n");
	*est= *mea;
ONCE_END

	vec3f_t lastHeading = est->heading;

	//printf("dt = %f, w = %f\n", dt, w);
	//printf("last heading= (%f, %f)\n", lastHeading.x, lastHeading.y); 
	//vec2fRot((vec2f_t*)&est->gyroHeading, (vec2f_t*)&lastHeading, w * dt);
	//printf("gyro = (%f, %f)\n", est->gyroHeading.x, est->gyroHeading.y); 
	static float lastC;
	//float coincidence = powf(vec3fDot(&mea->heading, &est->gyroHeading), 128);
	float da = vec3fAng(&mea->heading, &lastHeading);

	if(fabs(da) > 0.01){
		printf("da = %f w = %f\n", da, w);
		vec3Lerp(est->heading, lastHeading, mea->heading, w/da);//coincidence);
	}
	//est->heading = est->gyroHeading;

	//assert(!isnan(coincidence));
/*
	if(fabs(lastC - coincidence) > 0.001){
		lastC = coincidence;
		printf("w = %f, %f\n", w, coincidence);
	}
*/
}
//-----------------------------------------------------------------------------
int senUpdate(fusedObjState_t* body)
{
	float dt = SYS.timeUp - body->lastMeasureTime;
	objectState_t *measured  = &body->measured;
	objectState_t *estimated = &body->estimated;

	imuUpdateState(FD_IMU, &body->imu);

	estimateHeading(body, dt);

	if(gpsHasNewReadings()){
		vec3f_t lastPos = measured->position;

		// assign new ements
		vec3f_t* velLin = &measured->velocity.linear;
		body->hasGpsFix = gpsGetReadings(&measured->position, velLin);
		vec3Sub(*velLin, measured->position, lastPos);
		vec3Scl(*velLin, *velLin, dt);

		// since we now have measurements, reset the estimates
	//	*estimated = *measured;

		body->lastMeasureTime = SYS.timeUp;
		body->lastEstTime     = SYS.timeUp;
	}
	else
	{
		float dt = SYS.timeUp - body->lastEstTime;

		// integrate position using velocity
		vec3f_t* estVelLin = &estimated->velocity.linear;
		estimated->position.x += estVelLin->x * dt;
		estimated->position.y += estVelLin->y * dt;
		estimated->position.z += estVelLin->z * dt;

		// integrate IMU acceleration into velocity
		estVelLin->x += body->imu.adjReadings.linear.x * dt;
		estVelLin->y += body->imu.adjReadings.linear.y * dt;
		estVelLin->z += body->imu.adjReadings.linear.z * dt;

		body->lastEstTime = SYS.timeUp;
	}
	return 0;
}
