#ifndef AVC_DEC_STATES
#define AVC_DEC_STATES

#include "decisionState.h"

#ifdef __cplusplus
extern "C" {
#endif

enum stateTypes_e {
	STATE_TEMPLATE = 0,
	STATE_COUNT,
};

extern decision_t DEC_TEMPLATE;

extern void(*STATE_INITIALIZERS[STATE_COUNT])(void);

void decInitStates();

#ifdef __cplusplus
}
#endif
#endif
