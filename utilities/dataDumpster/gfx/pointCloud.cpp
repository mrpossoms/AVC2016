#include "pointCloud.h"
#include <stdlib.h>

using namespace gfx;

static vec3_t pcDefaultColoration(vec3* points, int point_i, vec3 min, vec3 max, float s)
{
	vec3 point = { points[point_i][0], points[point_i][1], points[point_i][2] };
	vec3_t color = {
		(point[0] - min[0]) / (max[0] - min[0]),
		(point[1] - min[1]) / (max[1] - min[1]),
		(point[2] - min[2]) / (max[2] - min[2]),
	};

	return color;
}
//------------------------------------------------------------------------------
void PointCloud::init()
{
	style		 = GL_POINTS;
	scaleFactor = 1;
	glPointSize(10);
	min[0] = min[1] = min[2] = -1;
	max[0] = max[1] = max[2] =  1;
	colorForPoint = pcDefaultColoration;
}
//------------------------------------------------------------------------------
PointCloud::PointCloud()
{
	count  = 1;
	points = (vec3*)calloc(1, sizeof(vec3) * count);
	shouldFree  = true;
	init();
}
//------------------------------------------------------------------------------
PointCloud::PointCloud(int pointCount)
{
	count		 = pointCount;
	points		= (vec3*)malloc(sizeof(vec3) * count);
	shouldFree  = true;
	init();
}
//------------------------------------------------------------------------------
PointCloud::PointCloud(vec3* pointData, int pointCount)
{
	count		 = pointCount;
	points		= pointData;
	shouldFree  = false;
	init();
}
//------------------------------------------------------------------------------
PointCloud::~PointCloud()
{
	if(shouldFree){
		free(points);
	}
}
//------------------------------------------------------------------------------
void PointCloud::addPoint(vec3 point)
{
		memcpy(points[current++], point, sizeof(vec3));
		current = current % count;
}
//------------------------------------------------------------------------------
void PointCloud::draw(Renderer* renderer)
{
	if(style == GL_LINES){
		glLineWidth(10);
	}

	glBegin(style);
	{
		if(style == GL_TRIANGLE_FAN)
		{
				glColor3f(0, 0, 0);
				glVertex3f(0, 0, 0);
		}

		for(int i = count; i--;){
			if(style == GL_LINES)
			{
				glColor3f(0, 0, 0);
				glVertex3f(0, 0, 0);
			}

			vec3_t pos = { points[i][0], points[i][1], points[i][2] }, t_pos;
			vec3_t color = colorForPoint(points, i, min, max, scaleFactor);

			// scale pos and rotate
			quat rotation;
			renderer->getRotation(rotation);
			vec3_scale(pos.v, pos.v, scaleFactor);
			quat_mul_vec3(t_pos.v, rotation, pos.v);

//			float attenuation = vec3_mul_inner(t_pos.v, t_pos.v) / 1;
//			vec3_scale(color.v, color.v, attenuation);

			glColor3f(color.x, color.y, color.z);
			// switch Z and Y since the data uses a right handed coord sys
			glVertex3f(pos.x, pos.y, pos.z);
		}
	}
	glEnd();

	if(style == GL_LINES){
		glLineWidth(1);
	}
}
