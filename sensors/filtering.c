#include "filtering.h"
#include "imu.h"
#include "gps.h"

#define IMU_SAMPLES 25
#define SENSOR_COUNT 4

static int create_filters(readingFilter_t* filters)
{
	vec3f_t* std_devs = &(filters->stdDevs.acc);

	for(int i = SENSOR_COUNT; i--;){
		kf_t* filter = (&filters->acc) + i;
		if(kfCreateFilter(filter, 3)){
			return -1;
		}

		// copy collected standard deviations into the measurement
		// covariance matrices for the filters
		for(int j = 3; j--;){
			for(int k = 3; k--;){
				if(j == k){
					filter->matR.col[j][k] = std_devs[i].v[j];
				}
			}
		}

		// set the proc-noise covariance matrix to an arbitrary value
		switch(i){
		case 0: // acc
		kfMatIdent(filter->matR);
		kfMatScl(filter->matR, filter->matR, 100);
		kfMatScl(filter->matQ, filter->matQ, 0.0001);
			break;
		case 1: // gyro
		kfMatScl(filter->matQ, filter->matQ, 0.01);
			break;
		case 2: // mag
		kfMatScl(filter->matQ, filter->matQ, 0.01);
			break;
		default:
		kfMatScl(filter->matQ, filter->matQ, 0.00001);
		}
	}

	return 0;
}

int sen_filters_init(int imu_fd, fusedObjState_t* body)
{
	sensorStatef_t samples[IMU_SAMPLES] = {};
	readingFilter_t* filters = &body->filters;
	vec3f_t heading;

	// wait to get a gps fix
	while(0 && gpsGetReadings(&samples[0].gps, &heading) != GPS_FIX){
		sleep(1);
		write(1, "-", 1);
	}

	// accumulate IMU samples
	for(int i = IMU_SAMPLES; i--;){
		sensorStatei_t raw = imuGetReadings(imu_fd);
		sensorStatef_t cal = imuApplyCalibration(&raw, body->imu.calMinMax);
		gpsGetReadings(&samples[i].gps, &heading);

		samples[i] = cal;

		// begin summation for the mean
		vec3Add(filters->means.acc, filters->means.acc, cal.acc);
		vec3Add(filters->means.gyro, filters->means.gyro, cal.gyro);
		vec3Add(filters->means.mag, filters->means.mag, cal.mag);
		vec3Add(filters->means.gps, filters->means.gps, samples[i].gps);

		// 5hz
		usleep(1000 * 200);
		if(i % 10) write(1, ".", 1);
	}

	// compute means
	vec3Scl(filters->means.acc,  filters->means.acc,  1 / (float)IMU_SAMPLES);
	vec3Scl(filters->means.gyro, filters->means.gyro, 1 / (float)IMU_SAMPLES);
	vec3Scl(filters->means.mag,  filters->means.mag,  1 / (float)IMU_SAMPLES);
	vec3Scl(filters->means.gps,  filters->means.gps,  1 / (float)IMU_SAMPLES);

	vec3f_t* std_devs = &(filters->stdDevs.acc);
	vec3f_t* mean = &(filters->means.acc);

	// compute variance
	for(int i = IMU_SAMPLES; i--;){
		vec3f_t* sensor = &(samples[i].acc);

		for(int j = SENSOR_COUNT; j--;){
			vec3f_t delta = {};
			vec3Sub(delta, sensor[j], mean[j]);
			vec3Had(delta, delta, delta); // delta = delta ^ 2

			vec3Add(std_devs[j], std_devs[j], delta);
		}
	}

	// compute standard deviations
	for(int i = SENSOR_COUNT; i--;){
		std_devs[i].x = sqrt(std_devs[i].x);
		std_devs[i].y = sqrt(std_devs[i].y);
		std_devs[i].z = sqrt(std_devs[i].z);
	}

	return create_filters(filters);
}
//-------------------------------------------------------------------
int sen_filter(fusedObjState_t* body)
{
	vec3f_t* cal      = &(body->measured.sensors.acc);
	vec3f_t* filtered = &(body->filtered.sensors.acc);
	kf_t*    filters  = &(body->filters.acc);

	for(int i = SENSOR_COUNT; i--;){
		kfPredict(filters + i, NULL);
		kfUpdate(filters + i, filtered[i].v, cal[i].v);
	}

	return 0;
}
