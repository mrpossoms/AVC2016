#include "servos.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>

int SERVO_FD;
int SERVOS[3];
int TRIM[3] = { 4, 0, 3 };

int ctrlInit()
{
	struct stat buf;

	if(!stat("/dev/servoblaster", &buf)){
		fprintf(stderr, "Servo driver already running\n");
	}	
	else if(system("servod --p1pins=37,38,36")){
		fprintf(stderr, "Failed to start servo driver\n");
		return -1;
	}

	SERVO_FD = open("/dev/servoblaster", O_WRONLY);

	if(SERVO_FD <= 0){
		return -2;
	}

	for(int i = 3; i--;)
	{
		ctrlSet(i, TRIM[i] + 50);
	}

	return 0;
}

int ctrlSet(int servo, int percent)
{
	if(!SERVO_FD) return -1;

	char str[12] = {};
	sprintf(str, "%d=%d%%\n", servo, percent + TRIM[servo]);
	write(SERVO_FD, str, strlen(str));
	SERVOS[servo] = percent;

	return 0;
}

int ctrlGet(int servo)
{
	return SERVOS[servo];
}
