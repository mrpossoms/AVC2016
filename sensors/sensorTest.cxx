#include <unistd.h>

#include "system.h"
#include "aggergate.h"

int main(int argc, char* argv[])
{
	senInit();

	while(1){
		senUpdate(&SYS.body);
		usleep(1000);
	}

	return 0;
}