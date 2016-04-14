#pragma once 

#include "renderer.h"
#include "pointCloud.h"

namespace gfx {

class Axes : public Drawable {
public:
	Axes();

	void draw(Renderer* renderer);
	vec3_t* accData;
private:
	PointCloud x, y, z;
};

}
