#include "gimbal.h"
#include <stdio.h>

using namespace gfx;

Gimbal::Gimbal()
{
	float dt = M_PI / 50.0;

	for(int i = 100; i--;){
		circle[i].x = cosf(dt * i);
		circle[i].y = sinf(dt * i);
		circle[i].z = 0;
	}

	angles.x = angles.y = angles.z = 0;
}

Gimbal::~Gimbal() {}

void Gimbal::draw(Renderer* renderer)
{
	mat4x4 I, T;
	mat4x4 rots[3];

	mat4x4_identity(I);

	mat4x4_rotate_Y(T, I, M_PI / 2.0f);
	mat4x4_rotate_X(rots[0], T, angles.x);

	mat4x4_rotate_Y(rots[1], T, angles.z);

	mat4x4_rotate_X(T, I, M_PI / 2.0f);
	mat4x4_rotate_Z(rots[2], T, angles.y);

	for(int i = 3; i--;){
		glPushMatrix();
		glMultMatrixf((float*)rots[i]);

		glBegin(GL_POINTS);
		{
			glColor3fv(I[i]);

			for(int j = 100; j--;){
				glVertex3fv(circle[j].v);
			}
		}
		glEnd();
		glPopMatrix();
	}
}
