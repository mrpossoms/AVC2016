#include "grid.h"

using namespace gfx;

Grid::Grid(int rows, int cols)
{
   this->rows = rows;
   this->cols = cols;
}
//-----------------------------------------------------------------------------
Grid::~Grid()
{

}
//-----------------------------------------------------------------------------
void Grid::draw(Renderer* renderer)
{
   float dx = 2 / (float)cols, dy = 2 / (float)rows;

   glBegin(GL_LINES);
   {
      glColor3f(1, 1, 1);
      for(int i = rows + 1; i--;){
         glVertex3f(dx * i - 1, 0, -1);
         glVertex3f(dx * i - 1, 0,  1);
      }

      for(int i = cols + 1; i--;){
         glVertex3f(-1, 0, dy * i - 1);
         glVertex3f( 1, 0, dy * i - 1);
      }
   }
   glEnd();
}
