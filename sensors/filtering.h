#ifndef AVC_SENSOR_FILTERING
#define AVC_SENSOR_FILTERING

#include "base/system.h"

#ifdef __cplusplus
extern "C"{
#endif

int sen_filters_init(int imu_fd, sensors_t* sensors);
int sen_filter(sensors_t* sensors);

#ifdef __cplusplus
}
#endif

#endif
