#include "diagnostics.h"
#include "system.h"

#include <stdio.h>

#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>

#define try(exp, errMsg){\
	int err = (exp);\
	if(err < 0){\
		fprintf(stderr, "Error(%d): '%s'\n", errno, errMsg);\
		return err;\
	}\
}\

static void* handler(void* params)
{
	int listenfd = *((int*)params); // listening fd
	int ci = 0;                     // connection index
	int connections[50] = {};
	int maxFd = 0, usedConnections = 0;
	struct timeval timeout = { 0, 10000 };

	while(1){
		int newConnection = accept(listenfd, NULL, NULL);

		if(newConnection > 0){
			connections[ci++] = newConnection;
			if(ci > usedConnections) ci = usedConnections;
			ci %= (sizeof(connections) / sizeof(int));
			if(newConnection > maxFd) maxFd = newConnection;
		}

		fd_set readFd;
		FD_ZERO(&readFd);
		for(int i = usedConnections; i--;){
			FD_SET(connections[ci], &readFd);
		}

		if(select(maxFd + 1, &readFd, 0, 0, &timeout) < 0){
			continue; // time-out occured
		}

		// for each connection that requested something
		for(int i = usedConnections; i--;){
			int fd = connections[i];
			if(FD_ISSET(fd, &readFd)){
				write(fd, &SYS.body, sizeof(fusedObjState_t));
			}
		}
	}

	return NULL;
}

int diagHost(short port)
{
	static int listenfd;
	struct sockaddr_in myAddr = {};
	pthread_t serverThread;

	// setup the address and port for the server
	myAddr.sin_family      = AF_INET;
	myAddr.sin_addr.s_addr = INADDR_ANY;
	myAddr.sin_port        = htons(port);

	try(listenfd = socket(AF_INET, SOCK_STREAM, 0), "Could not open socket");
	try(bind(listenfd, (const struct sockaddr*)&myAddr, sizeof(myAddr)), "Could not bind socket to address");
	try(listen(listenfd, 10), "Could not begin listening");

	// set the listen fd to non-blocking
	int flags = fcntl(listenfd, F_GETFD);
	fcntl(listenfd, F_SETFD, O_NONBLOCK | flags);

	try(pthread_create(&serverThread, NULL, handler, &listenfd), "Could not spawn diag server thread");

	return 0;
}