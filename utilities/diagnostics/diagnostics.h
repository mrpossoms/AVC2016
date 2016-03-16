#ifndef AVC_DIAGNOSTIC_SERVER
#define AVC_DIAGNOSTIC_SERVER
#include "base/system.h"

#ifdef __cplusplus
extern "C" {
#endif

int diagHost(short port);
int diagBlkBoxLog();

#ifdef __cplusplus
}
#endif

#endif
