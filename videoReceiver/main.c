#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <arpa/inet.h>
#include <sys/socket.h>

#include <GLFW/glfw3.h>

#include "stream.h"

// #define RENDER_DEMO

GLFWwindow* WIN;
char* frameBuffer = NULL;

int main(int argc, char* argv[])
{

	size_t   frameBufferSize;

	GLuint   frameTex;

	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port   = htons(1337),
		.sin_addr   = htonl(INADDR_ANY),
	};

	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	assert(bind(sock, (const struct sockaddr*)&addr, sizeof(addr)) >= 0);

    if (!glfwInit()){
            return -1;
    }

    WIN = glfwCreateWindow(640, 480, "AVC 2016", NULL, NULL);

    if (!WIN){
            glfwTerminate();
            return -2;
    }

    glfwMakeContextCurrent(WIN);
    glShadeModel(GL_FLAT);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

	glGenTextures(1, &frameTex);
	glBindTexture(GL_TEXTURE_2D, frameTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	int frameCount = 0;
	while(!glfwWindowShouldClose(WIN)){
		frameHeader_t header   = {};

#ifndef RENDER_DEMO
		int res = rxFrame(sock, &header, &frameBuffer);

#else
		header.width  = 128;
		header.height = 64;
#endif

		// printf("Frame %d (%d, %d) %zu\n", frameCount++, header.width, header.height, header.bytes);
		// printf("width %d height %d\n", header.width, header.height);

#ifdef RENDER_DEMO
		static int rand_fd;
		if(!rand_fd){
			rand_fd = open("/dev/random", O_RDONLY);
		}

		read(rand_fd, frameBuffer, frameBufferSize);
#endif

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

		glfwSwapBuffers(WIN);
	}

	return 0;
}