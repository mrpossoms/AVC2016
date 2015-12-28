#include <sys/socket.h>
#include <arpa/inet.h>
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>

#include "controls/servos.h"
#include "rc.h"

int main(int argc, char* argv[])
{
	int sock = socket(AF_INET, SOCK_DGRAM, 0);

	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port   = htons(1337),
		.sin_addr   = htonl(INADDR_ANY),
	};

	assert(!conInit());

	assert(sock > 0);
	bind(sock, (const struct sockaddr*)&addr, sizeof(addr));

	while(1){
		struct timeval timeout = { 0, 10000 };
		fd_set readFd;
		rcMessage_t msg = {};

		FD_ZERO(&readFd);
		FD_SET(sock, &readFd);

		if(select(sock + 1, &readFd, 0, 0, &timeout) <= 0){
			conSet(0, 50);
			conSet(1, 50);
			continue;
		}

		int bytes = recv(sock, &msg, sizeof(msg), 0);

		if(bytes == sizeof(msg)){
			conSet(0, msg.throttle);
			conSet(1, msg.steering);
		}

		usleep(1000);
	}

	return 0;
}
