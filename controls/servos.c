#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>

int SERVO_FD;

int conInit()
{
	struct stat buf;

	if(!stat("/dev/servoblaster", &buf)){
		fprintf(stderr, "Servo driver already running\n");
	}	
	else if(system("servod --p1pins=37,38")){
		fprintf(stderr, "Failed to start servo driver\n");
		return -1;
	}

	SERVO_FD = open("/dev/servoblaster", O_WRONLY);

	if(SERVO_FD <= 0){
		return -2;
	}

	return 0;
}


int conSet(int servo, int percent)
{
	char str[12] = {};
	sprintf(str, "%d=%d%\n", servo + 1, percent);
	printf("[%s]\n", str);
	write(SERVO_FD, str, strlen(str));
	return 0;
}

