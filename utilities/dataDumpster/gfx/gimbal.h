#pragma once
#include "gfx.h"

namespace gfx {

class Gimbal : public Drawable {
public:
	Gimbal();
	~Gimbal();

	void draw(Renderer* renderer);
	
	vec3_t angles;
private:
	vec3_t circle[100];
};

}
