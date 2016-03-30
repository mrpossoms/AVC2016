#pragma once

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>

#include "base/system.h"

namespace data {

class Client {
public:
   Client(const char* hostName, uint16_t port);
   ~Client();

   int connect();

   void(*onConnect)();
   void(*onData)(sysSnap_t snap);
private:
   struct sockaddr_in host;
   int sock;
   pthread_t pollingThread;
};

}
