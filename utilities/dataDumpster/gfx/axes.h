#pragma mark

#include "renderer.h"
#include "pointCloud.h"

namespace gfx {

class Axes : Drawable {
public:
	Axes();

	void draw(Renderer* renderer);
	vec3_t* accData;
private:
	PointCloud x, y, z;
};

}
