#include "types.h"

int vec3fIsNan(vec3f_t* v)
{
	return isnan(v->x) || isnan(v->y) || isnan(v->z);
}

int vec3dIsNan(vec3d_t* v)
{
	return isnan(v->x) || isnan(v->y) || isnan(v->z);
}

void vec3iEndianSwap(vec3i16_t* v)
{
	return;
	v->x = ntohs(v->x);
	v->y = ntohs(v->y);
	v->z = ntohs(v->z);
}

vec3f_t vec3fSub(vec3f_t* v1, vec3f_t* v2)
{
	vec3f_t res = {
		v1->x - v2->x,
		v1->y - v2->y,
		v1->z - v2->z,
	};
	return res;
}

vec3f_t vec3fScl(vec3f_t* v, float s)
{
	vec3f_t res = {
		v->x * s,
		v->y * s,
		v->z * s,
	};
	return res;
}

vec3f_t vec3fMul(vec3f_t* v1, vec3f_t* v2)
{
	vec3f_t res = {
		v1->x * v2->x,
		v1->y * v2->y,
		v1->z * v2->z,
	};
	return res;
}

float vec3fDot(vec3f_t* v1, vec3f_t* v2)
{
	return v1->x * v2->x + v1->y * v2->y +v1->z * v2->z;
}

float vec3fMag(vec3f_t* v)
{
	return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

vec3f_t vec3fNorm(vec3f_t* v)
{
	float mag = vec3fMag(v);

	vec3f_t res = {
		v->x / mag,
		v->y / mag,
		v->z / mag,
	};
	return res;
}

void vec2fRot(vec2f_t* r, vec2f_t* v, float theta)
{
	float c = cos(theta), s = sin(theta);
	r->x = c * v->x - s * v->y;
	r->y = s * v->x + c * v->y;
}

float vec3fAng(vec3f_t* a, vec3f_t* b)
{
	vec3f_t an = vec3fNorm(a);
	vec3f_t bn = vec3fNorm(b);
	return acosf(vec3fDot(&an, &bn));
}

double gauss(double mu, double sigma, double x)
{
	double var2 = (sigma * sigma) * 2;
	return (1.f / sqrt(var2 * M_PI)) * pow(M_E, -pow(x - mu, 2.f) / var2);
}
//-----------------------------------------------------------------------------
vec3d_t mtoll(vec3d_t* meters)
{
	vec3d_t ll = {
		mtodeg(meters->x),
		mtodeg(meters->y),
		mtodeg(meters->z)
	};

	return ll;
}
//-----------------------------------------------------------------------------
double mtodeg(float meters)
{
	const double dia = 6371000 * 2;
	return meters * 180 / (dia * M_PI);
}
//-----------------------------------------------------------------------------
double degtom(float degrees)
{
	const double dia = 6371000 * 2;
	return (degrees * (dia * M_PI)) / 180;
}
