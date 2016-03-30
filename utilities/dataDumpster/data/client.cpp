#include "client.h"

using namespace data;

typedef struct {
   int sock;
   void(*onConnect)();
   void(*onData)(sysSnap_t snap);
} clientEvents_t;

static void* pollingHandler(void* args)
{
   clientEvents_t* params = (clientEvents_t*)args;
   int sock = params->sock;
   ssize_t bytes = 0;
   sysSnap_t snapShot = {};

   if(params->onConnect){
      params->onConnect();
   }

   while(1){
      write(sock, "X", 1);

      for(int i = 1000; i && bytes < sizeof(sysSnap_t); i--){
         ioctl(sock, FIONREAD, &bytes);
         usleep(10000);
      }

      bytes = read(sock, &snapShot, sizeof(sysSnap_t));

      if(bytes == sizeof(sysSnap_t)){
         params->onData(snapShot);
      }

      usleep(500 * 1000);
   }
}

Client::Client(const char* hostName, uint16_t port)
{
   struct hostent* he = gethostbyname(hostName);

   if(!he){
      printf("Failed to resolve hostname\n");
      return;
   }

   host.sin_family = AF_INET;
   host.sin_port   = htons(port);
   unsigned char* addr = (unsigned char*)he->h_addr_list[0];
   memcpy((void*)&(host.sin_addr), addr, he->h_length);
}
//------------------------------------------------------------------------------
Client::~Client()
{
   close(sock);
}
//------------------------------------------------------------------------------
int Client::connect()
{
   sock = socket(AF_INET, SOCK_STREAM, 0);

   if(sock < 0){
      return -1;
   }

   if(::connect(sock, (struct sockaddr *)&host, sizeof(host)) < 0){
      close(sock);
      return -2;
   }

   uint32_t action = 0; // indicate that we want data streaming

   if(write(sock, &action, sizeof(action)) != sizeof(action)){
      return -3;
   }

   // start polling
   clientEvents_t* cbs = (clientEvents_t*)malloc(sizeof(clientEvents_t));
   cbs->onConnect = onConnect;
   cbs->onData = onData;
   return pthread_create(&pollingThread, 0, pollingHandler, (void*)cbs);
}
