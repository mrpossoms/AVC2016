#include "data.h"

vec3f_t DAT_DEPTH[SCANNER_RES]; 

vec3f_t DAT_MAG_RAW[SAMPLES];
vec3f_t DAT_MAG_CAL[SAMPLES];
vec3f_t DAT_MAG_EST[SAMPLES];
vec3f_t DAT_ACC_CAL;
sysSnap_t DAT_SNAPS[SAMPLES];
int DAT_CUR_IDX;
