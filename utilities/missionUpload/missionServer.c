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
#include <dirent.h>
#include <signal.h>

#include "base/system.h"

static char* AVC_PATH;

//-----------------------------------------------------------------------------
int fileCount(const char* path)
{
	int count = 0;
	DIR* dir = opendir(path);
	struct dirent *ep = NULL;

	if(dir){
			while((ep = readdir(dir))){
					if(ep->d_name[0] == '.') continue;
					++count;
			}
			closedir(dir);
	}

	return count;
}
//-----------------------------------------------------------------------------
void listBlackBoxLogs(int connfd)
{
	struct dirent *ep = NULL;
	char bbPath[256] = {};
	snprintf(bbPath, 256, "%sblackbox", AVC_PATH);

	// tell the client how many filenames to expect
	uint32_t files = fileCount(bbPath);
	DIR* dir = opendir(bbPath);

	// if we couldn't open the dir, tell them to expect nothing
	if(!dir){
		syslog(0, "Failed to open dir '%s'\n", bbPath);
		files = 0;
	}
	syslog(0, "sending %d files\n", files);

	write(connfd, &files, sizeof(uint32_t));

	// iterate over the whole list of files
	if(files)
	for(ep = readdir(dir); ep; ep = readdir(dir)){
		blkboxLog_t log = {};
		struct stat fileStat = {};

		// skip hidden files
		if(ep->d_name[0] == '.') continue;

		// get the size
		char wholePath[256] = {};
		snprintf(wholePath, 256, "%s/%s", bbPath, ep->d_name);
		stat(wholePath, &fileStat);

		int len = strlen(ep->d_name);
		memcpy(log.name, ep->d_name, len > sizeof(log.name) ? sizeof(log.name) : len);
		log.bytes = (uint32_t)fileStat.st_size;

		write(connfd, &log, sizeof(log));
	}
}
//-----------------------------------------------------------------------------
void downloadBlackBoxLog(int connfd)
{
	char bbPath[256] = {};
	char logName[64] = {};

	read(connfd, logName, sizeof(logName));
	snprintf(bbPath, 256, "%sblackbox/%s", AVC_PATH, logName);

	int logFd = open(bbPath, O_RDONLY);
	uint32_t logSize = (uint32_t)lseek(logFd, 0, SEEK_END) / sizeof(sysSnap_t); // how long?
	lseek(logFd, 0, SEEK_SET); // go back to the beginning

	// tell the client how many messages to expect
	write(connfd, &logSize, sizeof(logSize));

	// send the
	int hadGps = 0;
	size_t bytes = 0;
	sysSnap_t snap = {};
	while((bytes = read(logFd, &snap, sizeof(snap)))){
		if(snap.estimated.position.x == 0 && snap.estimated.position.y == 0){
			if(hadGps){
					syslog(0, "Log integrity uncertain");
			}
		}
		else{
			hadGps = 1;
		}

		write(connfd, &snap, sizeof(snap));
	}

	close(logFd);
}
//-----------------------------------------------------------------------------
void downloadMission(int connfd, const char* filepath)
{
	int missionfd = open(filepath, O_CREAT | O_WRONLY, 0666);

	gpsRouteHeader_t header = {};
	gpsWaypoint_t* waypoints = NULL;

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
	close(missionfd);
}
//-----------------------------------------------------------------------------
void killMissions()
{
        struct dirent *ep = NULL;

        // tell the client how many filenames to expect
        DIR* dir = opendir(AVC_PATH);

        // if we couldn't open the dir, tell them to expect nothing
        if(!dir){
                syslog(0, "Failed to open dir '%s'\n", AVC_PATH);
       		return;
	}

        // iterate over the whole list of files
        for(ep = readdir(dir); ep; ep = readdir(dir)){

                // skip hidden files
                if(ep->d_name[0] == '.') continue;

		int len = strlen(ep->d_name);
		if(memcmp(ep->d_name + (len - 4), ".pid", 4) == 0){
			ep->d_name[len - 4] = '\0';
			pid_t id = (pid_t)atoi(ep->d_name);
			kill(id, SIGKILL);
		}
        }

	closedir(dir);
}
//-----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	int listenfd = 0;
	struct sockaddr_in serv_addr = {};

	if(argc < 2){
		printf("Usage:\n\tmissionUpload [path to file]\n");
		return 1;
	}

	AVC_PATH = argv[1];

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
	sprintf(filepath, "%smission.gps", AVC_PATH);

	while(1){

		int connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
		fcntl(connfd, F_SETFD, fcntl(connfd, F_GETFD) | FD_CLOEXEC);

		syslog(0, "connected!");

		uint32_t action = -1;
		read(connfd, &action, sizeof(action));

		switch(action){
			case MISS_SRV_UPLOAD:
				syslog(0, "Uploading mission");
				downloadMission(connfd, filepath);
				break;
			case MISS_SRV_RUN:
				// run AVC
				syslog(0, "Running mission");
				system("/root/AVC2016/AVC /root/mission.gps --use-throttle");
				break;
			case MISS_SRV_BLKBOX_LIST:
				syslog(0, "Listing blackbox logs");
				listBlackBoxLogs(connfd);
				break;
			case MISS_SRV_BLKBOX_DOWNLOAD:
				syslog(0, "Downloading blackbox log");
				downloadBlackBoxLog(connfd);
				break;
			case MISS_SRV_KILL:
				syslog(0, "Terminating running missions");
				killMissions();
				break;
		}

		close(connfd);
	}

	return 0;
}
