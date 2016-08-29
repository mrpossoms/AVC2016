#pragma once

#include <GLFW/glfw3.h>
#include "linmath.h"

namespace gfx {

typedef struct { vec3_t col[3]; } mat3_t;

class Renderer {
public:
	Renderer(int w, int h);
	~Renderer();

	int  isOpen();
	void present();

	void getRotation(quat rotation);
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
