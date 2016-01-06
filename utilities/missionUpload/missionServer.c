#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <assert.h>
#include <strings.h>

#include "types.h"

int main(int argc, char *argv[])
{
	int listenfd = 0;
	struct sockaddr_in serv_addr = {}; 

	if(argc < 2){
		printf("Usage:\n\tmissionUpload [path to file]\n");
		return 1;
	}

	// spawn the child process
	pid_t pid = fork();

	if(pid){ // if i'm the parent then terminate
		printf("%s started\n", argv[0]);
		return 0;
	}

	// at this point the child continues
	pid_t leader = setsid(); // detach, create a new session

	if(umask(0)){
		return -1;
	}

	if(chdir(argv[1])){
		return -2;
	}

	// close open fds inherited from the parent
	for(int i = sysconf(_SC_OPEN_MAX); i--;){
//		close(i);
	}

	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	bzero(&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(1339); 

	assert(!bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr))); 
	assert(!listen(listenfd, 5)); 
	printf("Bound and listening\n");

	while(1){

		int connfd = accept(listenfd, (struct sockaddr*)NULL, NULL); 
		int missionfd = open(argv[1], O_CREAT | O_WRONLY, 0666);

		gpsRouteHeader_t header = {};
		gpsWaypoint_t* waypoints = NULL;

		printf("connected!\n");

 		read(connfd, &header, sizeof(header));
		write(missionfd, &header, sizeof(header));

 		waypoints = malloc(sizeof(gpsWaypoint_t) * header.waypoints);

		printf("\nRoute with %d waypoints\n", header.waypoints);

 		for(int i = 0; i < header.waypoints; ++i){
 			read(connfd, waypoints + i, sizeof(gpsWaypoint_t));
			vec3f_t pos = waypoints[i].location;
			printf("\t(%f, %f)\n", pos.x, pos.y);

			write(missionfd, waypoints + i, sizeof(gpsWaypoint_t));
 		}

 		free(waypoints);

		close(connfd);
		close(missionfd);
		sleep(1);
	}

	return 0;
}
