#include "client.h"

using namespace data;

static int resolve(clientThreadArgs_t* args)
{
	printf("Resolving '%s'\n", args->hostName);
	struct hostent* he = gethostbyname(args->hostName);

	if(!he){
		*args->state = ccs_failed;
		free(args);
		printf("Failed to resolve hostname\n");
		return -1;
	}

	unsigned char* addr = (unsigned char*)he->h_addr_list[0];
	memcpy((void*)&(args->host->sin_addr), addr, he->h_length);

	printf("Resolved to %d.%d.%d.%d\n", addr[0], addr[1], addr[2], addr[3]);

	return 0;
}

static void connectionFail(clientThreadArgs_t* args)
{
		close(args->sock);
		*args->state = ccs_failed;
		args->onConnect(ccs_failed);
		free(args);
}

static void* connectionHandler(void* args)
{
	clientThreadArgs_t* params = (clientThreadArgs_t*)args;
	sleep(1);

	if(resolve(params)){
		connectionFail(params);
		return NULL;
	}

	int sock = params->sock = socket(AF_INET, SOCK_STREAM, 0);
	if(sock < 0){
		printf("Failed to open socket\n");
		connectionFail(params);
		return NULL;
	}

	if(::connect(sock, (struct sockaddr *)params->host, sizeof(struct sockaddr_in)) < 0){
		printf("Failed to establish connection\n");
		connectionFail(params);
		return NULL;
	}

	uint32_t action = 0; // indicate that we want data streaming
	if(write(sock, &action, sizeof(action)) < sizeof(action)){
		printf("Failed to establish intention\n");
		connectionFail(params);
		return NULL;
	}

	if(params->onConnect){
		*params->state = ccs_connected;
		params->onConnect(ccs_connected);
	}

	// polling loop
	while(1){
		sysSnap_t snapShot = {};
		int bytes = 0;
		write(sock, "X", 1);

		for(int i = 1000; i && bytes < sizeof(sysSnap_t); i--){
			ioctl(sock, FIONREAD, &bytes);
			usleep(10000);
		}

		bytes = read(sock, &snapShot, sizeof(sysSnap_t));

		if(bytes >= sizeof(sysSnap_t)){
			params->onData(snapShot);
		}

		usleep(10 * 1000);
	}

	return NULL;
}
//------------------------------------------------------------------------------
Client::Client(const char* hostName, uint16_t port)
{
	host.sin_family = AF_INET;
	host.sin_port	= htons(port);

	this->hostName = hostName;
}
//------------------------------------------------------------------------------
Client::~Client()
{
	close(sock);
}
//------------------------------------------------------------------------------
int Client::connect()
{
	clientThreadArgs_t* cbs = (clientThreadArgs_t*)malloc(sizeof(clientThreadArgs_t));

	// start polling
	cbs->onConnect = onConnect;
	cbs->onData = onData;
	cbs->sock = sock;
	cbs->host = &host;
	cbs->state = &state;
	cbs->hostName = hostName;

	state = ccs_connecting;

	return pthread_create(&connectionThread, 0, connectionHandler, (void*)cbs);
}
