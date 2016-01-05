#include "agents.h"

agent_t* AGENTS_ALL[] = {
	&AGENT_TEMPLATE,
};

void agentInitAgents()
{
	for(int i = sizeof(AGENTS_ALL) / sizeof(agent_t); i--;){
		AGENTS_ALL[i]->init();
	}
}

