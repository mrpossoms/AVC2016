#include "Basis.h"

using namespace gfx;

void Basis::draw(Renderer* renderer)
{
	const vec3_t c[] = {
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 }
	};

	if(!matrix) return;
	mat3_t m = *matrix;

	glLineWidth(4);

	glBegin(GL_LINES);
	{
		for(int i = 3; i--;){
			glColor3f(c[i].x, c[i].y, c[i].z);
			glVertex3f(0, 0, 0);
			glVertex3f(m.col[i].x, m.col[i].z, m.col[i].y);
		}
	}
	glEnd();
}
