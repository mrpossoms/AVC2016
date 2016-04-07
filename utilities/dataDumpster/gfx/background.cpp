#include "background.h"

using namespace gfx;

void Background::draw(Renderer* renderer)
{
   glDepthMask(GL_FALSE);

   glPushMatrix();
   glLoadIdentity();

   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
   glLoadIdentity();

   glBegin(GL_TRIANGLE_FAN);
   {
      glColor3ub(154, 206, 235);
      glVertex3f(-1, 1, 0);
      glVertex3f(1, 1, 0);

      glColor3ub(100, 149, 237);
      glVertex3f(1, -1, 0);
      glVertex3f(-1, -1, 0);
   }
   glEnd();

   glPopMatrix();
   glMatrixMode(GL_MODELVIEW);
   glPopMatrix();

   glDepthMask(GL_TRUE);
}
