#include "aggergate.h"

#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <libNEMA.h>

static int FD_IMU;
static kfMat_t ROT_MAT, TEMP_MAT[2];

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

	ROT_MAT = kfMatAlloc(3, 3);
	TEMP_MAT[0] = kfMatAlloc(3, 3);
	TEMP_MAT[1] = kfMatAlloc(3, 3);

	if(calProfile){
		int calFd = open(calProfile, O_RDONLY);

		if(calFd <= 0 || imuLoadCalibrationProfile(calFd, &SYS.body.imu)){
			close(calFd);
			return -3;
		}

		close(calFd);
	}

	if(imuSetup(FD_IMU, &SYS.body.imu)){
		printf("IMU stat collection failed.\n");
		return -4;
	}

	printf("IMU stats collected.\n");

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

static void estimateHeading(float dt)
{
	fusedObjState_t* body = &SYS.body;
	objectState_t *mea= &body->measured;
	objectState_t *est= &body->estimated;

	// use the accelerometer's up/down vector to
	// help the system determine how the vehicle is
	// resting on the ground. This vector is used as
	// the basis for the Z axis in the vehicle's body
	// reference frame
	vec3f_t heading = body->imu.cal.mag;
	vec3f_t up      = vec3fNorm(&body->imu.cal.acc);
	vec3f_t forward = { 0, 1, 0 };

	if(!vec3fIsNan(&up) && vec3fMag(&up) <= LIL_G){

		kfVecCross(ROT_MAT.col[0], up.v, forward.v, 3);      // left
		memcpy(ROT_MAT.col[2], &up, sizeof(up));             // up
		kfVecCross(ROT_MAT.col[1], ROT_MAT.col[0], up.v, 3); // forward

		vec3fScl((vec3f_t*)ROT_MAT.col[0], -1);

		// rotate the mag vector back into the world frame
		kfMatCpy(TEMP_MAT[0], ROT_MAT);
		kfMat3Inverse(ROT_MAT, TEMP_MAT[0], TEMP_MAT[1]);
		kfMatMulVec(body->imu.cal.mag.v, ROT_MAT, heading.v, 3);
	}

	// Use the gyro's angular velocity to help correlate the
	// change in heading according to the magnetometer with
	// the apparent rate of vehicle rotation
	{
		// use the gyro to estimate how confident we should be in the magnetometer's
		// current measured heading
		float w = body->imu.raw.gyro.z / -32000.0f;

		// update the heading according to magnetometer readings
		mea->heading.x = -body->imu.filtered.mag.x;
		mea->heading.y = -body->imu.filtered.mag.y;
		mea->heading.z =  body->imu.filtered.mag.z;
		mea->heading = vec3fNorm(&mea->heading);

		if(vec3fIsNan(&mea->heading)) return;

		ONCE_START
		*est= *mea;
		ONCE_END

		vec3f_t lastHeading = est->heading;
		float da = vec3fAng(&mea->heading, &lastHeading);

		if(fabs(da) > 0.0001){
			float coincidence = fabs(w) / da;
			if(coincidence < 0) coincidence = 0;

			vec3Lerp(est->heading, lastHeading, mea->heading, coincidence);
		}
		//est->heading = est->gyroHeading;
		// est->heading = mea->heading;
	}

	// grab the bearing that the GPS module has
	// determined, use the land speed as an inverse weight
	//  for interpolation between the GPS heading and the mag / gyro heading
/*
	{
		float C = cosf(GPS_STATE.Bearing), S = sinf(GPS_STATE.Bearing);
		vec3f_t gpsHeading = { S, C, 0 };
		float p = 1.0f / (GPS_STATE.Speed + 0.0001);

		p = p > 1 ? 1 : p;
		vec3Lerp(est->heading, gpsHeading, est->heading, p);
	}
*/
}
//-----------------------------------------------------------------------------
int senUpdate(fusedObjState_t* body)
{
	float dt = SYS.timeUp - body->lastMeasureTime;
	objectState_t *measured  = &body->measured;
	objectState_t *estimated = &body->estimated;

	if(imuUpdateState(FD_IMU, &body->imu, SYS.magCal)) return -1;
	estimateHeading(dt);

	if(gpsHasNewReadings()){
		vec3f_t lastPos = measured->position;

		// assign new ements
		vec3f_t* velLin = &measured->velocity.linear;
		body->hasGpsFix = gpsGetReadings(&measured->position, velLin);
		vec3Sub(*velLin, measured->position, lastPos);
		vec3Scl(*velLin, *velLin, dt);

		estimated->position = measured->position;

		body->lastMeasureTime = SYS.timeUp;
		body->lastEstTime     = SYS.timeUp;
	}
	else
	{
		estimated->position = measured->position;

		// integrate position using velocity
		vec3f_t* estVelLin = &estimated->velocity.linear;

		// integrate IMU acceleration into velocity
		for(int i = 3; i--;){
			estVelLin->v[i] += body->imu.filtered.acc.v[i] * dt;
		}

		body->lastEstTime = SYS.timeUp;
	}
	return 0;
}
