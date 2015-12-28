#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>

#include <GLFW/glfw3.h>

#include "sensors/imu.h"
#include "stream.h"
#include "comms/protocol.h"
#include "utilities/rc.h"

// #define RENDER_DEMO

GLFWwindow* WIN;
char* frameBuffer = NULL;
struct sockaddr_in HOST;
depthWindow_t DEPTHS;

int RC_SOCK;
rcMessage_t RC_STATE;
int RC_NEW_DATA;

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_LEFT){
    	RC_STATE.steering = action == GLFW_RELEASE ? 50 : 25;
    }
    if (key == GLFW_KEY_RIGHT){
    	RC_STATE.steering = action == GLFW_RELEASE ? 50 : 75;
    }
     
    if (key == GLFW_KEY_UP){
    	RC_STATE.throttle = action == GLFW_RELEASE ? 50 : 55;
    }
    if (key == GLFW_KEY_DOWN){
    	RC_STATE.throttle = action == GLFW_RELEASE ? 50 : 45;
    }

    printf("%d %d\n", RC_STATE.throttle, RC_STATE.steering);
    RC_NEW_DATA = 1;
}

static void* rcWorker(void* args)
{
	RC_SOCK = socket(AF_INET, SOCK_DGRAM, 0);

	while(1){
		if(RC_NEW_DATA){
			struct sockaddr_in peer = HOST;
			peer.sin_port = htons(1338);

			sendto(
				RC_SOCK,
				&RC_STATE,
				sizeof(RC_STATE),
				0,
				(const struct sockaddr*)&peer,
				sizeof(peer)
			);
			printf("Sent!\n");
			RC_NEW_DATA = 0;
		}
	}
}

static int rxProcessorTracking(int sock, struct sockaddr_in* peer)
{
	depthWindow_t window;
	socklen_t len = sizeof(HOST);
	size_t windowSize = sizeof(depthWindow_t);

	printf("Tracking message %d\n", sock);

	size_t bytes = recvfrom(
		sock,
		&window,
		windowSize,
		0,
		(struct sockaddr*)peer,
		&len
	);

	printf("Bytes read %zu expected %zu\n", bytes, windowSize);
	assert(bytes == windowSize);

	return 0;
}

static int rxProcessorFrame(int sock, struct sockaddr_in* peer)
{
	frameHeader_t header   = {};
	int res = rxFrame(sock, &header, &frameBuffer);

	glTexImage2D(
		GL_TEXTURE_2D,
		0,
		GL_LUMINANCE, // one color channel
		header.width,
		header.height,
		0, // no border
		GL_LUMINANCE,
		GL_UNSIGNED_BYTE,
		frameBuffer
	);

	return 0;
}

static int rxProcessorDepths(int sock, struct sockaddr_in* peer)
{
	socklen_t addrLen = sizeof(HOST);
	struct sockaddr sender = {};

	size_t bytes = recvfrom(
		sock,
		&DEPTHS,
		sizeof(DEPTHS),
		0,
		&sender,
		&addrLen
	);

	return 0;
}

static void setupGL()
{
	glShadeModel(GL_SMOOTH);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}

static void createTexture(GLuint* tex)
{
	glGenTextures(1, tex);
	glBindTexture(GL_TEXTURE_2D, *tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

static int oneOK;
int main(int argc, char* argv[])
{
	size_t    frameBufferSize;
	GLuint    frameTex;
	pthread_t rcThread;

	if (!glfwInit()){
		return -1;
	}

	WIN = glfwCreateWindow(640, 480, "AVC 2016", NULL, NULL);

	if (!WIN){
		glfwTerminate();
		return -2;
	}

	glfwMakeContextCurrent(WIN);
	setupGL();
	createTexture(&frameTex);

	commInitClient(argv[1], 1337, &HOST);
	commRegisterRxProc(MSG_VIDEO, rxProcessorFrame);
	commRegisterRxProc(MSG_TRACKING, rxProcessorDepths);
	printf("size %d\n", commSend(MSG_GREETING, NULL, 0, &HOST));

	pthread_create(&rcThread, NULL, rcWorker, NULL);
	glfwSetKeyCallback(WIN, key_callback);

	int frameCount = 0;
	while(!glfwWindowShouldClose(WIN)){

	int res = 1;
#ifndef RENDER_DEMO
	res = commListen();

	if(!res) oneOK = 1;
#endif

	if(res && !oneOK){
		static int rand_fd;
		frameHeader_t header;

		header.width = 128;
		header.height = 64;

		if(!rand_fd){
			rand_fd = open("/dev/random", O_RDONLY);
		}

		if(!frameBuffer){
			frameBufferSize = header.width * header.height;
			frameBuffer = malloc(frameBufferSize);
		}

		read(rand_fd, frameBuffer, frameBufferSize);		

		glTexImage2D(
			GL_TEXTURE_2D,
			0,
			GL_LUMINANCE, // one color channel
			header.width,
			header.height,
			0, // no border
			GL_LUMINANCE,
			GL_UNSIGNED_BYTE,
			frameBuffer
		);

	}

		glClear(GL_COLOR_BUFFER_BIT);
		glEnable(GL_TEXTURE_2D);
		glBegin(GL_QUADS);
			glVertex2f( 1,  1);
			glTexCoord2f(0, 0);

			glVertex2f(-1,  1);
			glTexCoord2f(0, 1);

			glVertex2f(-1, -1);
			glTexCoord2f(1, 1);

			glVertex2f( 1, -1);
			glTexCoord2f(1, 0);
		glEnd();

		glPointSize(10);
		glDisable(GL_TEXTURE_2D);
		glBegin(GL_POINTS);
		for(int i = DEPTHS.detectedFeatures; i--;){
			glColor3f(DEPTHS.depth[i].z / 100.0f, 0.1f, 0);
			glVertex2f(DEPTHS.depth[i].x / (float)SHRT_MAX, -DEPTHS.depth[i].y / (float)SHRT_MAX);
		}
		glEnd();

		glfwPollEvents();
		glfwSwapBuffers(WIN);
	}

	return 0;
}
