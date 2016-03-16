#ifndef AVC_DEC_STATE
#define AVC_DEC_STATE

#include "base/system.h"

#ifdef __cplusplus
extern "C" {
#endif

struct decision_agent;
typedef float (*utilFunc_t)(struct decision_agent* currentState, void* args);

struct decision_agent{
	utilFunc_t              utility;
	float                   utilityValue;
	struct decision_agent** adjStates;
	int                     adjCount;
	void  (*init)();
	void* (*action)(struct decision_agent* lastState, void* args);
	void  (*stimulate)(float weight, struct decision_agent* stimulator);
};

typedef struct decision_agent agent_t;

int         agentAddAdjState(agent_t* dst, agent_t* adj);
agent_t* agentNextCandidate(agent_t* current, void* args);

#ifdef __cplusplus
}
#endif

#endif
