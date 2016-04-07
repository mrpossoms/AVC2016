#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include "gfx.h"
#include "data.h"
#include "axes.h"

using namespace gfx;
using namespace data;

PointCloud* magCloud;

static vec3_t pcRawColor(vec3 point, vec3 min, vec3 max, float s)
{
   vec3_t color = {
      1,
      (point[1] - min[1]) / (max[1] - min[1]),
      (point[2] - min[2]) / (max[2] - min[2]),
   };

   return color;
}

static vec3_t pcCalColor(vec3 point, vec3 min, vec3 max, float s)
{
   vec3_t color = {
      (point[0] - min[0]) / (max[0] - min[0]),
      1,
      (point[2] - min[2]) / (max[2] - min[2]),
   };

   return color;
}

static vec3_t pcEstColor(vec3 point, vec3 min, vec3 max, float s)
{
   vec3_t color = {
      (point[0] - min[0]) / (max[0] - min[0]),
      (point[1] - min[1]) / (max[1] - min[1]),
      1
   };

   return color;
}

static void onData(sysSnap_t snap)
{
   DAT_SNAPS[DAT_CUR_IDX] = snap;

   vec3 rawMag = { snap.imu.raw.mag.x, snap.imu.raw.mag.y, snap.imu.raw.mag.z };

   memcpy(DAT_MAG_CAL + DAT_CUR_IDX, &snap.imu.cal.mag, sizeof(vec3));
   memcpy(DAT_MAG_EST + DAT_CUR_IDX, &snap.estimated.heading, sizeof(vec3));
   memcpy(DAT_MAG_RAW + DAT_CUR_IDX, rawMag, sizeof(vec3));
   memcpy(&DAT_ACC_CAL, &snap.imu.cal.acc, sizeof(vec3));

   DAT_CUR_IDX++;
   DAT_CUR_IDX %= SAMPLES;
}

static void onConnect(int res)
{
   if(res == ccs_connected){
      magCloud->points = (vec3*)DAT_MAG_CAL;
      magCloud->style = GL_POINTS;
      printf("Connected\n");
   }
   else{
      printf("failed %d\n", res);
   }
}

int main(int argc, char* argv[])
{
   Renderer win(640, 480);
   Grid grid(10, 10);
   Background background;

   vec3 randomPoints[1000];
   for(int i = 1000; i--;){
      randomPoints[i][0] = ((random() % 1024) / 512.f) - 1;
      randomPoints[i][1] = ((random() % 1024) / 512.f) - 1;
      randomPoints[i][2] = ((random() % 1024) / 512.f) - 1;
      vec3_norm(randomPoints[i], randomPoints[i]);
   }

   Axes accPlot;
   accPlot.accData = (vec3_t*)&DAT_ACC_CAL;

   PointCloud rawMagCloud((vec3*)DAT_MAG_RAW, SAMPLES);
   rawMagCloud.scaleFactor = 1.0f / (float)0x1FFF;
   rawMagCloud.colorForPoint = pcRawColor;

   magCloud = new PointCloud((vec3*)randomPoints, 1000);
   magCloud->style = GL_LINES;
   magCloud->colorForPoint = pcCalColor;

   PointCloud estMagCloud((vec3*)DAT_MAG_EST, 1000);
   estMagCloud.colorForPoint = pcEstColor;

   Client client(argv[1], 1340);
   client.onConnect = onConnect;
   client.onData = onData;

   printf("Connecting...");
   client.connect();

   float t = 0;
   while(win.isOpen()){
      background.draw(&win);

      grid.draw(&win);
      if(client.state == ccs_connecting){
         mat4x4 scl = {}, ident = {};
         mat4x4_identity(ident);
         mat4x4_scale(scl, ident, t);
         scl[3][3] = 1;
         glPushMatrix();
         glMultMatrixf((float*)scl);
         magCloud->draw(&win);
         glPopMatrix();

         t += (1 - t) * 0.1;
         if(t > 0.99) t = 0;
      }
      else{
         magCloud->draw(&win);
         rawMagCloud.draw(&win);
         estMagCloud.draw(&win);
      }
      accPlot.draw(&win);

      win.present();
   }

   return 0;
}
