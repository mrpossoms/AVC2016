#include "renderer.h"
#include <assert.h>

using namespace gfx;

static int IS_MOUSE_DOWN;
static float CAM_DX, CAM_DY;
static float CAM_X, CAM_Y, CAM_Z = -2;
static mat4x4* VIEW;

static void cursorMoved(GLFWwindow* win, double x, double y)
{
	if(IS_MOUSE_DOWN){
		CAM_DX = x - CAM_X; CAM_DY = y - CAM_Y;
	}
	else{
		CAM_DX = 0; CAM_DY = 0;
	}

	CAM_X = x; CAM_Y = y;
}
//------------------------------------------------------------------------------
static void mouseButton(GLFWwindow* win, int btn, int act, int mod)
{
	IS_MOUSE_DOWN = act == GLFW_PRESS;
}
//------------------------------------------------------------------------------
void scroll(GLFWwindow* win, double dx, double dy)
{
	CAM_Z += dy * 0.01f;
	mat4x4_translate(*VIEW, 0, -0.5, CAM_Z);
}
//------------------------------------------------------------------------------
Renderer::Renderer(int w, int h)
{
	assert(glfwInit());
	assert((win = glfwCreateWindow(w, h, "Data-Dumpster", NULL, NULL)));
	glfwMakeContextCurrent(win);

	glfwSetCursorPosCallback(win, cursorMoved);
	glfwSetMouseButtonCallback(win, mouseButton);
	glfwSetScrollCallback(win, scroll);

	mat4x4_perspective(proj, M_PI / 2, w / (float)h, 0.01, 100);
	mat4x4_translate(view, 0, -0.5, CAM_Z);

	VIEW = &view;

	glClearColor(0, 0, 0, 1);

	glEnable(GL_DEPTH_TEST);

	quat_identity(rotation);
}
//-----------------------------------------------------------------------------
Renderer::~Renderer()
{
	glfwTerminate();
}
//-----------------------------------------------------------------------------
int Renderer::isOpen()
{
	mat4x4 m_rot;
	quat q_delta, q_temp[2];

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf((float*)proj);

	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf((float*)view);

	// create the delta quaternion
	quat_from_axis_angle(q_temp[0], 0, 1, 0, CAM_DX / 300);
	quat_from_axis_angle(q_temp[1], 1, 0, 0, CAM_DY / 300);
	quat_mul(q_delta, q_temp[0], q_temp[1]);

	// multiply it with the accumulated rotation quat
	quat_mul(q_temp[0], q_delta, rotation);
	memcpy(rotation, q_temp[0], sizeof(rotation));

	// create a rotation matrix from that which GL can consume
	mat4x4_from_quat(m_rot, rotation);

	CAM_DX = CAM_DY = 0; // stop us from spinning unwantedly :)

	glPushMatrix();
	glMultMatrixf((float*)m_rot);

	return !glfwWindowShouldClose(win);
}
//-----------------------------------------------------------------------------
void Renderer::present()
{
	glPopMatrix();
	glfwSwapBuffers(win);
	glfwPollEvents();
}
//-----------------------------------------------------------------------------
void Renderer::getRotation(quat rotation)
{
	for(int i = 4; i--;)
		rotation[i] = this->rotation[i];
}
