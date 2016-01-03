#ifndef AVC_DEC_STATE
#define AVC_DEC_STATE

#ifdef __cplusplus
extern "C" {
#endif

struct decision_state;
typedef float (*utilFunc_t)(struct decision_state* currentState, void* args);

struct decision_state{
	utilFunc_t              utility;
	float                   utilityValue;
	struct decision_state** adjStates;
	int                     adjCount;
	void* (*action)(struct decision_state* lastState, void* args);	
};

typedef struct decision_state decision_t;

int         decAddAdjState(decision_t* dst, decision_t* adj);
decision_t* decNextCandidate(decision_t* current, void* args);

#ifdef __cplusplus
}
#endif

#endif
