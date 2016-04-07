#include "agents.h"

agent_t* AGENTS_ALL[] = {
	&AGENT_TEMPLATE,
	&AGENT_STEERING,
	&AGENT_ROUTING,
	&AGENT_THROTTLE,
	&AGENT_CRASH_DETECTOR,
};

void agentInitAgents()
{
	printf("Initializing Agents...");
	for(int i = sizeof(AGENTS_ALL) / sizeof(agent_t); i--;){
		AGENTS_ALL[i]->init();
	}
	printf("OK\n");
}
