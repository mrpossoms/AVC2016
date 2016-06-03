#include "aggergate.h"

#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <libNEMA.h>

#include "filtering.h"

static int FD_I2C;
static kfMat_t ROT_MAT, TEMP_MAT[2];

#define ONCE_START {\
	static int once;\
	if(!once){\
	once = 1;\

#define ONCE_END } }

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

	printf("Initializing I2C bus...");
	if(!openSensor(imuDevice, &FD_I2C, O_RDWR)){
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

		if(calFd <= 0 || imuLoadCalibrationProfile(calFd, &SYS.sensors.imu)){
			close(calFd);
			return -3;
		}

		close(calFd);
	}
	printf("OK!\n");

	printf("Initalizing filters"); fflush(stdout);
	if(sen_filters_init(FD_I2C, &SYS.sensors)){
		printf("Sensor filter init failed.\n");
		return -4;
	}
	printf("OK!\n");

	printf("IMU stats collected.\n");

	return 0;
}
//-----------------------------------------------------------------------------
int senShutdown()
{
	if(gpsShutdown()) return -1;
	if(close(FD_I2C)) return -2;

	return 0;
}
//-----------------------------------------------------------------------------
static void estimateHeading(float dt, sensors_t* sensors, pose_t* pose)
{
	vec3f_t lastHeading = pose->heading;
	vec3f_t heading;
	//  = &SYS.sensors;
	// sensorStatef_t* mea = &sensors->measured;
	// sensorStatef_t* filtered_sensors = &sensors->filtered;

	{
		// update the heading according to magnetometer readings
		pose->heading.x = sensors->filtered.mag.x;
		pose->heading.y = sensors->filtered.mag.y;
		pose->heading.z = sensors->filtered.mag.z;
		heading = pose->heading = vec3fNorm(&pose->heading);

		if(vec3fIsNan(&pose->heading)) return;

	}

	// apply the transform, use the old matrix if it hasn't been
	// updated this cycle
	kfMatMulVec(
		pose->heading.v,
		ROT_MAT,
		heading.v,
		3
	);

	pose->heading.z = 0; // z means nothing to us at this point
	pose->heading = vec3fNorm(&pose->heading);
	pose->heading.x *= -1; pose->heading.y *= -1;

	// ONCE_START
	// *est= *mea;
	// ONCE_END

	// Use the gyro's angular velocity to help correlate the
	// change in heading according to the magnetometer with
	// the apparent rate of vehicle rotation
	{
		vec3f_t gyroHeading = {};

		// use the gyro to estimate how confident we should be in the magnetometer's
		// current measured heading
		float w = sensors->filtered.gyro.z / -32000.0f;

		vec2fRot((vec2f_t*)&gyroHeading, (vec2f_t*)&lastHeading, w * dt);

	//	float p = pow(vec3Dot(gyroHeading, mea->heading), 4);

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
static void new_reference_frame(sensors_t* sensors, pose_t* pose)
{
	// use the accelerometer's up/down vector to
	// help the system determine how the vehicle is
	// resting on the ground. This vector is used as
	// the basis for the Z axis in the vehicle's body
	// reference frame

	// TODO weight between ROT_MAT.col[2] and up. up's strength
	// should be inversely proportional to the it's magnitude
	// probably at some high degree function

	// vec3f_t heading = vec3fNorm(&sensors->filtered.mag);
	vec3f_t up      = vec3fNorm(&sensors->filtered.acc);
	vec3f_t forward = { 0, 1, 0 };

	if(!vec3fIsNan(&up) && vec3fMag(&sensors->filtered.acc) <= LIL_G * 1.05f){
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

		// rotate the mag vector back into the world frame
		kfMatCpy(TEMP_MAT[0], ROT_MAT);
		kfMatTranspose(ROT_MAT, TEMP_MAT[0]);

		// keep a copy of the last rot matrix for debugging purposes
		for(int i = 3; i--;){
			pose->accFrame[i] = *((vec3f_t*)ROT_MAT.col[i]);
		}
	}

}
//-----------------------------------------------------------------------------
static void estimate_pose(sensors_t* sens, pose_t* pose)
{
	// figure out which direction are we pointing
	estimateHeading(SYS.dt, sens, pose);

	// add dead reckoning component to the system location
	vec3f_t pos = sens->measured.gps;
	vec3f_t delta = vec3fScl(&pose->heading, sens->measured.enc_dist);
	vec3Add(pose->pos, pos, delta);

	// update the reference frame and rotation matrix
	new_reference_frame(sens, pose);
}
//-----------------------------------------------------------------------------
int senUpdate(sensors_t* sen)
{
	if(imuUpdateState(FD_I2C, &sen->imu, SYS.magCal)){
		printf("imuUpdateState() failed\n");
		return -1;
	}

	// read the wheel rotations from the rotary encoder
	i2cReqBytes(FD_I2C, 0x08, 0, &sen->imu.raw.enc_ticks, 1);

	// if(gpsHasNewReadings()){
	// 	vec3f_t lastPos = measured->sensors.gps;
	//
	// 	// assign new ements
	// 	vec3f_t* velLin = &measured->velocity.linear;
	// 	sen->hasGpsFix = gpsGetReadings(&measured->sensors.gps, velLin);
	// 	vec3Sub(*velLin, measured->sensors.gps, lastPos);
	// 	vec3Scl(*velLin, *velLin, dt);
	//
	// 	estimated->sensors.gps = measured->sensors.gps;
	//
	// 	sen->lastMeasureTime = SYS.timeUp;
	//
	// }

	// do filtering
	sen_filter(sen);

	// update the pose estimation
	estimate_pose(&SYS.sensors, &SYS.pose);

	sen->lastEstTime = SYS.timeUp;
	return 0;
}
