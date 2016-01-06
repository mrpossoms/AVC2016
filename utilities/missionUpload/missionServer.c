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
#include <syslog.h>

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
	openlog("missionUpload", 0, 0);

	umask(022);

	if(chdir(argv[1])){
		syslog(0, "%s: failed to change dir.", argv[0]);
		return -1;
	}

	// close open fds inherited from the parent
	for(int i = sysconf(_SC_OPEN_MAX); i--;){
		close(i);
	}

	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	if(listenfd < 0){
		syslog(0, "%s: failed to open socket.", argv[0]);
		return -2;
	}

	bzero(&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(1339); 

	if(bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr))){
		syslog(0, "%s: failed bind to port 1339.", argv[0]);
		return -3;
	} 
	assert(!listen(listenfd, 5)); 
	syslog(0, "%s: Bound and listening", argv[0]);

	char filepath[128];
	sprintf(filepath, "%smission.gps", argv[1]);

	while(1){

		int connfd = accept(listenfd, (struct sockaddr*)NULL, NULL); 
		int missionfd = open(filepath, O_CREAT | O_WRONLY, 0666);

		gpsRouteHeader_t header = {};
		gpsWaypoint_t* waypoints = NULL;

		syslog(0, "connected!");

 		read(connfd, &header, sizeof(header));
		write(missionfd, &header, sizeof(header));

 		waypoints = malloc(sizeof(gpsWaypoint_t) * header.waypoints);

		syslog(0, "\nRoute with %d waypoints\n", header.waypoints);

 		for(int i = 0; i < header.waypoints; ++i){
 			read(connfd, waypoints + i, sizeof(gpsWaypoint_t));
			vec3f_t pos = waypoints[i].location;
			syslog(0, "\t(%f, %f)\n", pos.x, pos.y);

			write(missionfd, waypoints + i, sizeof(gpsWaypoint_t));
 		}

 		free(waypoints);

		close(connfd);
		close(missionfd);
	}

	return 0;
}
