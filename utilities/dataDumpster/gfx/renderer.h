#pragma once

#include <GLFW/glfw3.h>
#include "linmath.h"

namespace gfx {

class Renderer {
public:
	Renderer(int w, int h);
	~Renderer();

	int  isOpen();
	void present();
private:
	GLFWwindow* win;
	mat4x4 view, proj;

	quat rotation;
};
//-----------------------------------------------------------------------------
class Drawable {
public:
	virtual void draw(Renderer* renderer) = 0;
};

}
