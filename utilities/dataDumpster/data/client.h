#pragma once

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>

#include "base/system.h"

namespace data {

typedef enum {
	ccs_none = 0,
	ccs_connecting,
	ccs_failed,
	ccs_connected
} clientConState_t;

typedef struct {
	int sock;
	const char* hostName;
	struct sockaddr_in* host;
	clientConState_t* state;
	void(*onConnect)(int res);
	void(*onData)(sysSnap_t snap);
} clientThreadArgs_t;

class Client {
public:
	Client(const char* hostName, uint16_t port);
	~Client();

	int connect();

	void(*onConnect)(int res);
	void(*onData)(sysSnap_t snap);

	clientConState_t state;
private:
	struct sockaddr_in host;
	int sock;
	const char* hostName;
	pthread_t pollingThread, connectionThread;
};

}
