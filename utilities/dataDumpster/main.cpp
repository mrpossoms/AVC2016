#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <assert.h>

#include "gfx.h"
#include "data.h"
#include "axes.h"

using namespace gfx;
using namespace data;

const vec3_t RED = { 1, 0, 0 };
const vec3_t BLACK = { 0, 0, 0 };


PointCloud* magCloud;
mat3_t ACC_BASIS_MAT;
vec3_t HEADING;

vec3_t pcRed(vec3* points, int point_i, vec3 min, vec3 max, float s)
{
	return RED;
}

vec3_t pcBlack(vec3* points, int point_i, vec3 min, vec3 max, float s)
{
	return BLACK;
}

vec3_t regionColor(unsigned char theta)
{
	unsigned char rgb[3] = {};
	unsigned char hsv[3] = {
		theta,
		255,
		255
	};
	unsigned char region, remainder, p, q, t;

	region = hsv[0] / 43;
	remainder = (hsv[0] - (region * 43)) * 6;

	p = 0;
	q = (hsv[2] * (255 - ((hsv[1] * remainder) >> 8))) >> 8;
	t = (hsv[2] * (255 - ((hsv[1] * (255 - remainder)) >> 8))) >> 8;

	switch (region)
	{
		 case 0:
			  rgb[0] = hsv[2]; rgb[1] = t; rgb[2] = p;
			  break;
		 case 1:
			  rgb[0] = q; rgb[1] = hsv[2]; rgb[2] = p;
			  break;
		 case 2:
			  rgb[0] = p; rgb[1] = hsv[2]; rgb[2] = t;
			  break;
		 case 3:
			  rgb[0] = p; rgb[1] = q; rgb[2] = hsv[2];
			  break;
		 case 4:
			  rgb[0] = t; rgb[1] = p; rgb[2] = hsv[2];
			  break;
		 default:
			  rgb[0] = hsv[2]; rgb[1] = p; rgb[2] = q;
			  break;
	}

	vec3_t color = {rgb[0] / 255.f, rgb[1] / 255.f, rgb[2] / 255.f};
	return color;
}

static vec3_t pcRawColor(vec3* points, int point_i, vec3 min, vec3 max, float s)
{
	vec3 point = { points[point_i][0], points[point_i][1], points[point_i][2] };
	vec3_t color = {
		1,
		(point[1] - min[1]) / (max[1] - min[1]),
		(point[2] - min[2]) / (max[2] - min[2]),
	};

	return color;
}

static vec3_t pcObsColor(vec3* points, int point_i, vec3 min, vec3 max, float s)
{
	return regionColor(DAT_OBS[point_i] * 20);
}

static vec3_t pcCalColor(vec3* points, int point_i, vec3 min, vec3 max, float s)
{
	vec3 point = { points[point_i][0], points[point_i][1], points[point_i][2] };
	vec3_t color = {
		(point[0] - min[0]) / (max[0] - min[0]),
		1,
		(point[2] - min[2]) / (max[2] - min[2]),
	};

	return color;
}

static vec3_t pcEstColor(vec3* points, int point_i, vec3 min, vec3 max, float s)
{
	vec3 point = { points[point_i][0], points[point_i][1], points[point_i][2] };
	vec3_t color = {
		(point[0] - min[0]) / (max[0] - min[0]),
		(point[1] - min[1]) / (max[1] - min[1]),
		1
	};

	return color;
}

static float vec3_std_dev(vec3f_t* arr, int len)
{
	float var = 0;
	vec3f_t mu = {};
	float w = 1 / (float)len;

	for(int i = len; i--;)
	{
		vec3f_t temp = arr[i];
		vec3_scale(temp.v, temp.v, w);
		vec3_add(mu.v, mu.v, temp.v);
	}

	for(int i = len; i--;)
	{
		vec3f_t temp;
		vec3_sub(temp.v, arr[i].v, mu.v);

		var += vec3_mul_inner(temp.v, temp.v) * w;
	}

	return sqrtf(var);
}

static void onData(sysSnap_t snap)
{

	DAT_CUR_IDX++;
	DAT_CUR_IDX %= SAMPLES;

	DAT_SNAPS[DAT_CUR_IDX] = snap;

	vec3 rawMag = { snap.sensors.filtered.mag.x, snap.sensors.filtered.mag.y, snap.sensors.filtered.mag.z };

	memcpy(DAT_MAG_CAL + DAT_CUR_IDX, snap.sensors.filtered.mag.v, sizeof(vec3));
	memcpy(DAT_MAG_EST + DAT_CUR_IDX, snap.pose.heading.v, sizeof(vec3));
	memcpy(DAT_MAG_RAW + DAT_CUR_IDX, rawMag, sizeof(vec3));
	memcpy(&DAT_ACC_CAL, snap.sensors.filtered.acc.v, sizeof(vec3));
	memcpy(HEADING.v, snap.pose.heading.v, sizeof(vec3));

	memcpy(&ACC_BASIS_MAT, snap.pose.accFrame, sizeof(snap.pose.accFrame));

	scn_datum_t d = snap.lastDepth;
	DAT_OBS[d.index] = d.obs_ind;
	vec3Sub(DAT_DEPTH[d.index], d.location, snap.pose.pos);
	vec3Scl(DAT_DEPTH[d.index], DAT_DEPTH[d.index], 1000000);


	quat rot = {};
	quat_from_axis_angle(rot, 1, 0, 0, M_PI / 2);
	quat_mul_vec3(DAT_DEPTH[d.index].v, rot, DAT_DEPTH[d.index].v);

	vec3Sub(DAT_OBS_NEAREST, snap.nearest_obs.centroid, snap.pose.pos);
	vec3Scl(DAT_OBS_NEAREST, DAT_OBS_NEAREST, 1000000);
	quat_mul_vec3(DAT_OBS_NEAREST.v, rot, DAT_OBS_NEAREST.v);


	if(d.index == 0)
	{
		printf("Scanner stddev: %f\n", vec3_std_dev(DAT_DEPTH, SCANNER_RES));
	}
}

static void onConnect(int res)
{
	if(res == ccs_connected){
		magCloud->points = (vec3*)DAT_MAG_CAL;
		magCloud->style = GL_POINTS;
		printf("Connected\n");
	}
	else{
		printf("failed %d\n", res);
	}
}

int main(int argc, char* argv[])
{
	Renderer win(640, 480);
	Grid grid(10, 10);
	Background background;

	vec3 randomPoints[1000];
	for(int i = 1000; i--;){
		randomPoints[i][0] = ((random() % 1024) / 512.f) - 1;
		randomPoints[i][1] = ((random() % 1024) / 512.f) - 1;
		randomPoints[i][2] = ((random() % 1024) / 512.f) - 1;
		vec3_norm(randomPoints[i], randomPoints[i]);
	}

	Axes accPlot;
	accPlot.accData = (vec3_t*)&DAT_ACC_CAL;

	Basis accBasis;
	accBasis.matrix = &ACC_BASIS_MAT;

	PointCloud rawMagCloud((vec3*)DAT_MAG_RAW, SAMPLES);
	rawMagCloud.scaleFactor = 1.0f / (float)0x1FFF;
	rawMagCloud.colorForPoint = pcRawColor;

	magCloud = new PointCloud((vec3*)randomPoints, 1000);
	magCloud->style = GL_LINES;
	magCloud->colorForPoint = pcCalColor;

	PointCloud estMagCloud((vec3*)DAT_MAG_EST, 1000);
	estMagCloud.colorForPoint = pcEstColor;

	PointCloud currentHeading((vec3*)HEADING.v, 1);
	currentHeading.colorForPoint = pcEstColor;
	currentHeading.style = GL_LINES;

	PointCloud scannerCloud((vec3*)DAT_DEPTH, SCANNER_RES);
	scannerCloud.colorForPoint = pcObsColor;
	scannerCloud.style = GL_TRIANGLE_FAN;

	PointCloud obstaclePoint((vec3*)&DAT_OBS_NEAREST, 1);
	obstaclePoint.colorForPoint = pcBlack;

	for(int i = SCANNER_RES; i--;)
	{
		DAT_DEPTH[i].x = 0;//cosf(i * ((M_PI / 2) / SCANNER_RES) - (M_PI / 4));
		DAT_DEPTH[i].z = 0;//sinf(i * ((M_PI / 2) / SCANNER_RES) - (M_PI / 4));
	}

	Gimbal gimbal;

	if(argv[1] == NULL)
	{
		printf("Error: Please provide an IP address\n");
		return -1;
	}

	Client client(argv[1], 1340);
	client.onConnect = onConnect;
	client.onData = onData;

	printf("Connecting...");
	client.connect();

	std::vector<Drawable*> drawables;

	drawables.push_back(&grid);
	drawables.push_back(magCloud);
	drawables.push_back(&rawMagCloud);
	drawables.push_back(&estMagCloud);
	drawables.push_back(&accBasis);
	drawables.push_back(&accPlot);
	drawables.push_back(&gimbal);
	drawables.push_back(&currentHeading);
	drawables.push_back(&scannerCloud);
	drawables.push_back(&obstaclePoint);

	float t = 0;
	while(win.isOpen()){
		static float last_time;
		if(last_time == 0) last_time = glfwGetTime();
		float dt = glfwGetTime() - last_time;

		vec3f_t delta = DAT_SNAPS[DAT_CUR_IDX].sensors.filtered.gyro;
		vec3_scale(delta.v, delta.v, dt);
		vec3_add(gimbal.angles.v, gimbal.angles.v, delta.v);

		background.draw(&win);

		if(client.state == ccs_connecting){
			mat4x4 scl = {}, ident = {};
			mat4x4_identity(ident);
			mat4x4_scale(scl, ident, t);
			scl[3][3] = 1;
			glPushMatrix();
			glMultMatrixf((float*)scl);
			magCloud->draw(&win);
			glPopMatrix();

			t += (1 - t) * 0.1;
			if(t > 0.99) t = 0;
		}
		else{
			for(int i = 0; i < drawables.size(); i++){
				drawables[i]->draw(&win);
			}
		}

		win.present();
	}

	return 0;
}
