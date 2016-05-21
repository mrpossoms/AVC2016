#include "aggergate.h"

#include <assert.h>
#include <string.h>
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

	printf("Allocating filter matrices...");
	ROT_MAT = kfMatAlloc(3, 3);
	TEMP_MAT[0] = kfMatAlloc(3, 3);
	TEMP_MAT[1] = kfMatAlloc(3, 3);
	printf("OK!\n");

	printf("Loading calibration values...");
	if(calProfile){
		int calFd = open(calProfile, O_RDONLY);

		if(calFd <= 0 || imuLoadCalibrationProfile(calFd, &SYS.body.imu)){
			close(calFd);
			return -3;
		}

		close(calFd);
	}
	printf("OK!\n");

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
	{
		// update the heading according to magnetometer readings
		mea->heading.x = body->imu.filtered.mag.x;
		mea->heading.y = body->imu.filtered.mag.y;
		mea->heading.z = body->imu.filtered.mag.z;
		mea->heading = vec3fNorm(&mea->heading);

		if(vec3fIsNan(&mea->heading)) return;

	}

	// use the accelerometer's up/down vector to
	// help the system determine how the vehicle is
	// resting on the ground. This vector is used as
	// the basis for the Z axis in the vehicle's body
	// reference frame

	// TODO weight between ROT_MAT.col[2] and up. up's strength
	// should be inversely proportional to the it's magnitude
	// probably at some high degree function

	vec3f_t heading = mea->heading;
	vec3f_t up      = vec3fNorm(&body->imu.filtered.acc);
	vec3f_t forward = { 0, 1, 0 };
	
	if(!vec3fIsNan(&up) && vec3fMag(&body->imu.filtered.acc) <= LIL_G * 1.05f){
		kfVecCross(ROT_MAT.col[0], up.v, forward.v, 3);      // left
		memcpy(ROT_MAT.col[2], &up, sizeof(up));             // up
		kfVecCross(ROT_MAT.col[1], ROT_MAT.col[0], up.v, 3); // forward

		// mirror the column vectors for the basis over the ground plane
		for(int j = 3; j--;){
			float v[3] = {
				ROT_MAT.col[j][0],
				ROT_MAT.col[j][1],
				ROT_MAT.col[j][2]
			};
			float p  = 2.f * v[2]; 
			for(int i=3; i--;){
				ROT_MAT.col[j][i] = v[i] - (i == 2 ? p : 0);
			}
		}

		// normalize all column vectors 
		//kfMatNormalize(ROT_MAT, ROT_MAT);

		// rotate the mag vector back into the world frame
		kfMatCpy(TEMP_MAT[0], ROT_MAT);
		kfMatTranspose(ROT_MAT, TEMP_MAT[0]);

		// keep a copy of the last rot matrix for debugging purposes
		for(int i = 3; i--;){
			est->accFrame[i] = *((vec3f_t*)ROT_MAT.col[i]);
		}
	}

	// apply the transform, use the old matrix if it hasn't been
	// updated this cycle
	kfMatMulVec(
		mea->heading.v,
		ROT_MAT,
		heading.v,
		3
	);

	mea->heading.z = 0; // z means nothing to us at this point
	mea->heading = vec3fNorm(&mea->heading);
	mea->heading.x *= -1; mea->heading.y *= -1;

	ONCE_START
	*est= *mea;
	ONCE_END

	// Use the gyro's angular velocity to help correlate the
	// change in heading according to the magnetometer with
	// the apparent rate of vehicle rotation
	{
		vec3f_t lastHeading = est->heading;
		vec3f_t gyroHeading = {};

		// use the gyro to estimate how confident we should be in the magnetometer's
		// current measured heading
		float w = body->imu.filtered.gyro.z;

		vec2fRot((vec2f_t*)&gyroHeading, (vec2f_t*)&lastHeading, w * SYS.dt);
		
	//	float p = pow(vec3Dot(gyroHeading, mea->heading), 4);


		est->heading = mea->heading;
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

	if(imuUpdateState(FD_IMU, &body->imu, SYS.magCal)){
		printf("imuUpdateState() failed\n");
		return -1;
	}
	
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
