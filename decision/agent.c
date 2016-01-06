#include "agent.h"
#include <stdlib.h>

int agentAddAdjState(agent_t* dst, agent_t* adj)
{
	if(!dst) return -1;

	if(dst->adjStates){
		dst->adjStates = (agent_t**)malloc(sizeof(agent_t*));
		dst->adjCount = 0;

		if(!dst->adjStates) return -2;
	}
	else{
		++dst->adjCount;
		dst->adjStates = (agent_t**)realloc(dst->adjStates, sizeof(agent_t*) * dst->adjCount);
		
		if(!dst->adjStates) return -3;
	}

	dst->adjStates[dst->adjCount - 1] = adj;

	return 0;
}

static int utilEval(agent_t** a, agent_t** b)
{
	// sort in descending order
	if((*a)->utilityValue > (*b)->utilityValue) return -1;
	if((*a)->utilityValue < (*b)->utilityValue) return  1;

	return 0;  
}

agent_t* agentNextCandidate(agent_t* current, void* args)
{
	agent_t* candidate = current;
	float       candidateUtility = current->utility(NULL, args);

	// update all utility values
	for(int i = current->adjCount; i--;){
		agent_t* state = current->adjStates[i];
		state->utilityValue = state->utility(current, args);

		if(state->utilityValue > candidateUtility){
			candidateUtility = state->utilityValue;
			candidate        = state;
		}
	}

	// pick the highest utility
	return candidate;
}
