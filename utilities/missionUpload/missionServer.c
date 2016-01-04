#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <assert.h>

#include "types.h"

int main(int argc, char *argv[])
{
	int listenfd = 0;
	struct sockaddr_in serv_addr = {}; 

	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	bzero(&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(1339); 

	assert(!bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr))); 
	assert(!listen(listenfd, 5)); 
	printf("Bound and listening\n");

	while(1)
	{
		write(1, ".", 1);

		int connfd = accept(listenfd, (struct sockaddr*)NULL, NULL); 
		gpsRouteHeader_t header = {};
		gpsWaypoint_t* waypoints = NULL;

		printf("connected!\n");

 		read(connfd, &header, sizeof(header));

 		waypoints = malloc(sizeof(gpsWaypoint_t) * header.waypoints);
 		for(int i = 0; i < header.waypoints; ++i){
 			read(connfd, waypoints + i, sizeof(gpsWaypoint_t));
 		}

 		free(waypoints);

		close(connfd);
		sleep(1);
	}

	return 0;
}
