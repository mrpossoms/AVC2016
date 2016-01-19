#ifndef AVC_DEC_AGENTS
#define AVC_DEC_AGENTS

#include "agent.h"

#ifdef __cplusplus
extern "C" {
#endif

// agent decarations
extern agent_t AGENT_TEMPLATE;
extern agent_t AGENT_STEERING;
extern agent_t AGENT_ROUTING;
extern agent_t AGENT_THROTTLE;

// list of all agent objects
extern agent_t* AGENTS_ALL[];

void agentInitAgents();

#ifdef __cplusplus
}
#endif
#endif
