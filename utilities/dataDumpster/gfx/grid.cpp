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

	glLineWidth(4);
	glBegin(GL_LINES);
	{
		glColor3f(1, 0, 0);
		glVertex3f(0, 0, -1);
		glVertex3f(0, 0,  1);

		glColor3f(0, 1, 0);
		glVertex3f(-1, 0, 0);
		glVertex3f( 1, 0, 0);
	}
	glEnd();

	glLineWidth(1);
	glBegin(GL_LINES);
	{
		glColor3f(1, 1, 1);
		for(int i = rows + 1; i--;){
			float x = dx * i - 1;
			glVertex3f(x, 0, -1);
			glVertex3f(x, 0,  1);
		}

		for(int i = cols + 1; i--;){
			float y = dy * i - 1;
			glVertex3f(-1, 0, y);
			glVertex3f( 1, 0, y);
		}
	}
	glEnd();
}
