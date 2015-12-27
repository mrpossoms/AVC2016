#include "decisionState.h"
#include <stdlib.h>

int decAddAdjState(decision_t* dst, decision_t* adj)
{
	if(!dst) return -1;

	if(dst->adjStates){
		dst->adjStates = (decision_t**)malloc(sizeof(decision_t*));
		dst->adjCount = 0;

		if(!dst->adjStates) return -2;
	}
	else{
		++dst->adjCount;
		dst->adjStates = realloc(dst->adjStates, sizeof(decision_t*) * dst->adjCount);
		
		if(!dst->adjStates) return -3;
	}

	dst->adjStates[dst->adjCount - 1] = adj;

	return 0;
}

static int utilEval(decision_t** a, decision_t** b)
{
	// sort in descending order
	if((*a)->utilityValue > (*b)->utilityValue) return -1;
	if((*a)->utilityValue < (*b)->utilityValue) return  1;

	return 0;  
}

decision_t* decNextCandidate(decision_t* current, void* args)
{
	decision_t* candidate = current;
	float       candidateUtility = current->utility(NULL, args);

	// update all utility values
	for(int i = current->adjCount; i--;){
		decision_t* state = current->adjStates[i];
		state->utilityValue = state->utility(current, args);

		if(state->utilityValue > candidateUtility){
			candidateUtility = state->utilityValue;
			candidate        = state;
		}
	}

	// pick the highest utility
	return candidate;
}
