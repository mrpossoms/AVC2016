#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>

#include "gfx.h"
#include "data.h"

using namespace gfx;
using namespace data;

static void onData(sysSnap_t snap)
{
   DAT_SNAPS[DAT_CUR_IDX++] = snap;
   DAT_CUR_IDX %= SAMPLES;

   printf("Data %ld", random() % 0xFF);
}

int main(int argc, char* argv[])
{
   Renderer win(640, 480);
   Grid grid(10, 10);
   Background background;

   vec3 points[1000];
   for(int i = 1000; i--;){
      points[i][0] = ((random() % 1024) / 512.f) - 1;
      points[i][1] = ((random() % 1024) / 512.f) - 1;
      points[i][2] = ((random() % 1024) / 512.f) - 1;
      vec3_norm(points[i], points[i]);
   }

   PointCloud cloud(points, 1000);
   Client client("nect.me", 1340);

   cloud.style = GL_LINES;

   client.onConnect = NULL;
   client.onData = onData;

   printf("Connecting...");
   if(client.connect()){
      printf("Failed\n");
   }
   else{
      printf("Connected!\n");
   }

   while(win.isOpen()){
      background.draw(&win);

      grid.draw(&win);
      cloud.draw(&win);

      win.present();
   }

   return 0;
}
