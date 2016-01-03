#include "states.h"

void(*STATE_INITIALIZERS[STATE_COUNT])(void);

void decAllStates()
{
	for(int i = STATE_COUNT; i--;){
		STATE_INITIALIZERS[i]();
	}
}

