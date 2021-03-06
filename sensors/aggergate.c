#include "aggergate.h"

#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <libNEMA.h>

#include "filtering.h"
#include "scanner.h"
#include "servos.h"

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
int senInit(const char* i2c_dev, const char* gpsDevice, const char* calProfile)
{
	printf("Initializing GPS...");
	if(gpsInit(gpsDevice)){
		printf("Failed!\n");
		return -1;
	}
	printf("OK!\n");

	printf("Initializing I2C bus...");
	if(!openSensor(i2c_dev, &FD_I2C, O_RDWR)){
		printf("Failed!\n");
		return -2;
	}
	// tell the encoder to reset
	i2cSendByte(FD_I2C, 0x08, 0, 1);
	printf("OK!\n");

	// Initalize the scanner turret and range finder
	if(SYS.use_scanner)
	{
		printf("Initializing scanner...");
		if(scn_init(
			&SYS.sensors.scanner,
			30, 60,
			54 * M_PI / 180.f,  // 54 deg scan window
			0.1,	   // 20ms / tick
			30))       // far-plane, 10M
		{
			printf("Failed!\n");
			return -3;
		}
		else
		{
			printf("OK!\n");
		}
	}

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
			return -4;
		}

		close(calFd);
	}
	printf("OK!\n");

	printf("Initalizing filters"); fflush(stdout);
	if(sen_filters_init(FD_I2C, &SYS.sensors)){
		printf("Sensor filter init failed.\n");
		return -5;
	}

	printf("OK!\n");

	printf("IMU stats collected.\n");
	bzero(&SYS.sensors.measured, sizeof(sensorStatef_t));

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

	if(vec3fIsNan(&lastHeading)) return;

	// ONCE_START
	// *est= *mea;
	// ONCE_END

	// Use the gyro's angular velocity to help correlate the
	// change in heading according to the magnetometer with
	// the apparent rate of vehicle rotation
	{
		vec3f_t gyroHeading = {};
		float expected = sensors->mag_expected.len;
		float std_dev  = sensors->mag_expected.std_dev / 2;
		float mag = vec3fMag(&sensors->filtered.mag);
		float dev = fabs(expected - mag);


		lerp(sensors->mag_expected.len, sensors->mag_expected.len, mag, dt);

		//printf("Exp %f, Std.d %f, len: %f, ∆ %f\n", expected, std_dev, mag, expected - mag);

		//printf(dev <= std_dev ? "good\n" : "bad\n");

		// use the gyro to estimate how confident we should be in the magnetometer's
		// current measured heading
		float w = -sensors->measured.gyro.z;

		//if(fabs(w) < 0.08) w = 0;
		//if(dev > std_dev) dev = std_dev;

			//printf("G %f\n", w);
		float p = (dev / std_dev) + 0.75;//dev / std_dev;


		vec2fRot((vec2f_t*)&gyroHeading, (vec2f_t*)&lastHeading, w * dt);

		//float diff = pow(1.f - vec3Dot(gyroHeading, pose->heading), 1.f / 32.f);
		//p += diff;

		//printf("mag = %f, diff = %f, p = %f\n", mag, diff, p);


		//static int lp;
		int is_bad = p > 1;
		static time_t last_time;
		static float sys_hz;
		time_t now = time(NULL);


		if(dt > 0)
		{
			if(sys_hz == 0) sys_hz = 1 / dt;
			sys_hz = (sys_hz * .5) + (1 / dt) * .5;
		}

		if(is_bad)
		{
			p = 1;
			if(last_time != now)
			printf("bad %lx %fhz\n", now, sys_hz);
		}
		else
		{
			if(last_time != now)
			printf("good %lx %fhz\n", now, sys_hz);
		}

		last_time = now;

		//lp = is_bad;

		//sensors->mag_expected.len = sensors->mag_expected.len * (1 - pdt) + mag * pdt;

		//printf("gh = %f, %f\n", gyroHeading.x, gyroHeading.y);
		vec3Lerp(pose->heading, pose->heading, gyroHeading, p);

		pose->heading = vec3fNorm(&pose->heading);
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
static void estimate_pose(sensors_t* sens, pose_t* pose, int new_gps)
{
	static int updates;
	static float elapsed;
	static float last_enc_tick;

	// figure out which direction are we pointing
	estimateHeading(SYS.dt, sens, pose);

	// add dead reckoning component to the system location

	float dd = sens->measured.enc_dist_delta;
	vec3d_t delta = {
		(double)(pose->heading.x * dd),
		(double)(-pose->heading.y * dd),
		0.
	};

	if(dd > 0)
	{
		float sec_per_tick = elapsed - last_enc_tick;
		pose->vel.x = delta.x * dd / sec_per_tick;
		pose->vel.y = delta.y * dd / sec_per_tick;
		last_enc_tick = elapsed;
	}
	else
	{
		vec3f_t zero = {};
		vec3Lerp(pose->vel, pose->vel, zero, 1/200.f);
	}

	delta = mtoll(&delta); // convert to lat-lon

	if(!vec3dIsNan(&delta))
	{
		vec3Add(pose->pos, pose->pos, delta);
	}

	if(new_gps && 0){
		const double max = 1;//gauss(0, 6, 0);
		const double bias = 0.0001;

		vec3d_t gps = sens->measured.gps;
		double w[2] = {
			bias + gauss(pose->pos.x, 3, gps.x) / max,
			bias + gauss(pose->pos.y, 3, gps.y) / max,
		};

		if(isnan(pose->pos.x * pose->pos.y) ||
		   w[0] == 0 || w[1] == 0
		){
			printf("reset\n"), sleep(1);
			pose->pos.x = gps.x;
			pose->pos.y = gps.y;
			pose->pos.z = gps.z;
			w[0] = w[1] = 1;
		}

		//printf("pose.x=%f, gps.x=%f\n", pose->pos.x, gps.x);
		//printf("pose.y=%f, gps.y=%f\n", pose->pos.y, gps.y);
		printf("weights: %f %f elapsed %f sec\n", w[0], w[1], elapsed);

		//pose->pos.x = pose->pos.x * (1 - w[0]) + gps.x * w[0];
		//pose->pos.y = pose->pos.y * (1 - w[1]) + gps.y * w[1];

		//pose->pos = pos;

		//printf("Updates: %d\n", updates);
		updates = elapsed = 0;
		//printf("heading: %0.3f %0.3f %0.3f ", pose->heading.x, pose->heading.y, pose->heading.z);
	}

	elapsed += SYS.dt;

	++updates;

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
	uint8_t data[2] = {};
	ioctl(FD_I2C, I2C_SLAVE, 0x08);
	read(FD_I2C, data, 2);

	uint8_t ticks = data[0], dec_m = data[1];
	float m = dec_m;


	if(ticks){
		printf("%d %d\n", ticks, dec_m);
	}

	if(ticks == 255){
		ticks = 0;
	}

	if(SYS.use_scanner)
	{
		scn_update(&SYS.sensors.scanner, m * .1f);
	}

	const float diameter = 0.1; // meters
	float sign = ctrlGet(SERVO_THROTTLE) > 50 ? 1.f : -1.f;
	sen->imu.cal.enc_dist_delta = ticks * diameter * M_PI * sign;
	sen->imu.cal.enc_dist += sen->imu.cal.enc_dist_delta;

	// do filtering
	sen_filter(sen);

	int new_gps = 0;
	sen->measured = sen->imu.cal;
	if(gpsHasNewReadings()){
		 // assign new ements
		vec3f_t velLin = {};
		vec3d_t pos = {};
		sen->hasGpsFix = gpsGetReadings(&pos, &velLin);

		//printf("Coord %f, %f\n", pos.x, pos.y);
		sen->measured.gps = pos;
		new_gps = 1;
	}
	// update the pose estimation
	estimate_pose(sen, &SYS.pose, new_gps);

	sen->lastEstTime = SYS.timeUp;
	return 0;
}
