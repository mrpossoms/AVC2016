#include "renderer.h"

namespace gfx {

class PointCloud : Drawable {
public:
   PointCloud(int pointCount);
   PointCloud(vec3* pointData, int pointCount);
   ~PointCloud();

   void addPoint(vec3 point);
   void draw(Renderer* renderer);

   vec3_t(*colorForPoint)(vec3 point, vec3 min, vec3 max, float s);
   vec3 min, max;
   float scaleFactor;
   GLenum style;
   vec3* points;
   int count;
private:
   void init();

   int current;
   bool shouldFree;
};

}
