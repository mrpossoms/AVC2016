#include "agents.h"

agent_t* AGENTS_ALL[] = {
	&AGENT_TEMPLATE,
	&AGENT_STEERING,
	&AGENT_ROUTING,
};

void agentInitAgents()
{
	for(int i = sizeof(AGENTS_ALL) / sizeof(agent_t); i--;){
		AGENTS_ALL[i]->init();
	}
}

