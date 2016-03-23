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
	vec3f_t heading = body->imu.adjReadings.mag;
	vec3f_t up      = vec3fNorm(&body->imu.adjReadings.linear);
	vec3f_t forward = { 0, 1, 0 };


	if(!vec3fIsNan(&up) && vec3fMag(&up) <= LIL_G){
		//printf("up = (%f, %f, %f)\n", heading.x, heading.y, heading.z);

		kfVecCross(ROT_MAT.col[0], up.v, forward.v, 3); // left
		memcpy(ROT_MAT.col[2], &up, sizeof(up));        // up
		kfVecCross(ROT_MAT.col[1], ROT_MAT.col[0], up.v, 3);   // forward

		ROT_MAT.col[0][0] *= -1;
		ROT_MAT.col[0][1] *= -1;
		ROT_MAT.col[0][2] *= -1;

		// rotate the mag vector back into the world frame
		kfMatCpy(TEMP_MAT[0], ROT_MAT);
		kfMat3Inverse(ROT_MAT, TEMP_MAT[0], TEMP_MAT[1]);		
		kfMatMulVec(body->imu.adjReadings.mag.v, ROT_MAT, heading.v, 3);
	}

	// use the gyro to estimate how confident we should be in the magnetometer's
	// current measured heading
	float w = body->imu.adjReadings.rotational.z / -32000.0f;

	// update the heading according to magnetometer readings
	mea->heading.x = -body->imu.adjReadings.mag.x;
	mea->heading.y = -body->imu.adjReadings.mag.y;
	mea->heading.z =  body->imu.adjReadings.mag.z;
	mea->heading = vec3fNorm(&mea->heading);

	if(vec3fIsNan(&mea->heading)) return;

ONCE_START
	*est= *mea;
ONCE_END

	vec3f_t lastHeading = est->heading;

	//printf("dt = %f, w = %f\n", dt, w);
	//printf("last heading= (%f, %f)\n", lastHeading.x, lastHeading.y);
	//vec2fRot((vec2f_t*)&est->gyroHeading, (vec2f_t*)&lastHeading, w * dt);
	//printf("gyro = (%f, %f)\n", est->gyroHeading.x, est->gyroHeading.y);
	//float coincidence = powf(vec3fDot(&mea->heading, &est->gyroHeading), 128);
	float da = vec3fAng(&mea->heading, &lastHeading);

	if(fabs(da) > 0.0001){
		float coincidence = fabs(w) / da;
		if(coincidence < 0) coincidence = 0;

//		printf("da = %f w = %f\n", da, w);
		vec3Lerp(est->heading, lastHeading, mea->heading, coincidence);
	}
	//est->heading = est->gyroHeading;
	est->heading = mea->heading;

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

	imuUpdateState(FD_IMU, &body->imu, SYS.magCal);
	estimateHeading(body, dt);

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
		estVelLin->x += body->imu.adjReadings.linear.x * dt;
		estVelLin->y += body->imu.adjReadings.linear.y * dt;
		estVelLin->z += body->imu.adjReadings.linear.z * dt;

		body->lastEstTime = SYS.timeUp;
	}
	return 0;
}
