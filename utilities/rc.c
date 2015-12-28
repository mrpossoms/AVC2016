#include <sys/socket.h>
#include <arpa/inet.h>
#include <assert.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "controls/servos.h"
#include "rc.h"

int main(int argc, char* argv[])
{
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	int yes = 1;
	setsockopt(sock, AF_INET, SO_REUSEADDR, &yes, sizeof(yes));	
	setsockopt(sock, AF_INET, SO_REUSEPORT, &yes, sizeof(yes));
	errno = 0;

	struct sockaddr_in addr = { };

	addr.sin_family      = AF_INET;
	addr.sin_port        = htons(2048);
	addr.sin_addr.s_addr = (INADDR_ANY);

	printf("Setup\n");

	assert(!conInit());

	assert(sock > 0);

	int res = bind(sock, (const struct sockaddr*)&addr, sizeof(addr));
	printf("%d %d\n", res, errno);
	
	assert(res >= 0);

	while(1){
		struct timeval timeout = { 1, 0 };
		fd_set readFd;
		rcMessage_t msg = { 0, 50, 50 };

		FD_ZERO(&readFd);
		FD_SET(sock, &readFd);


		if(select(sock + 1, &readFd, 0, 0, &timeout) <= 0){
			conSet(0, 50);
			conSet(1, 50);
			continue;
		}


		int bytes = recv(sock, &msg, sizeof(msg), MSG_WAITALL);

		printf("Got %d bytes\n");

		if(bytes == sizeof(msg)){
			conSet(1, msg.throttle);
			conSet(0, msg.steering);
		}

		usleep(1000);
	}

	return 0;
}
