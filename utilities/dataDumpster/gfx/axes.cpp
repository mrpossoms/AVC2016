#include "axes.h"

using namespace gfx;

Axes::Axes()
{
   x.style = GL_LINES;
   y.style = GL_LINES;
   z.style = GL_LINES;

   x.scaleFactor = y.scaleFactor = z.scaleFactor = 0.1f;
}

void Axes::draw(Renderer* renderer)
{
   x.points[0][0] = (accData)->x;
   y.points[0][1] = (accData)->y;
   z.points[0][2] = (accData)->z;

   x.draw(renderer);
   y.draw(renderer);
   z.draw(renderer);
}
