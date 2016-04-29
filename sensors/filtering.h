#ifndef AVC_SENSOR_FILTERING
#define AVC_SENSOR_FILTERING

#include "base/system.h"

#ifdef __cplusplus
extern "C"{
#endif

int sen_filters_init(int imu_fd, fusedObjState_t* body);
int sen_filter(fusedObjState_t* body);

#ifdef __cplusplus
}
#endif

#endif
