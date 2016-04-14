#pragma once

#include "renderer.h"

namespace gfx {

class Basis : public Drawable {
public:
	void draw(Renderer* renderer);
	mat3_t* matrix;
private:
};

}
