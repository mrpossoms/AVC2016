#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <assert.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#include "controls/servos.h"
#include "rc.h"

int main(int argc, char* argv[])
{
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in addr = { };	
	addr.sin_family      = AF_INET;
	addr.sin_port        = htons(1338);
	addr.sin_addr.s_addr = INADDR_ANY;

	fcntl(sock, F_SETFD, fcntl(sock, F_GETFD) | FD_CLOEXEC);
	printf("Setup using port %d\n", ntohs(addr.sin_port));

	assert(sock > 0);

	int res = bind(sock, (const struct sockaddr*)&addr, sizeof(addr));
	printf("%d %d\n", res, errno);
	
	assert(res >= 0);
	assert(!ctrlInit());
	
	while(1){
		struct timeval timeout = { 1, 0 };
		fd_set readFd;
		rcMessage_t msg = { 0, 50, 50 };

		FD_ZERO(&readFd);
		FD_SET(sock, &readFd);


		if(select(sock + 1, &readFd, 0, 0, &timeout) <= 0){
			ctrlSet(0, 50);
			ctrlSet(1, 50);
			continue;
		}


		int bytes = recv(sock, &msg, sizeof(msg), 0);
		printf("Got something\n");
		if(bytes == sizeof(msg)){
			ctrlSet(1, msg.throttle);
			ctrlSet(0, msg.steering);
		}

		usleep(1000);
	}

	return 0;
}
