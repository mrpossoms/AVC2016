#include "test.h"
#include "sensors/gps.h"
#include "sensors/scanner.c"

#define ROUTE_LEN 100

#define RAND_F (random() % 2048 / 1024.f)

gpsWaypointCont_t route[ROUTE_LEN];
scn_obstacle_t int_obs, obs;

vec3d_t normal()
{
	vec3d_t v = { RAND_F, RAND_F, RAND_F };
	double mag = sqrt(vec3Dot(v, v));
	vec3Scl(v, v, 1 / mag);
	return v;
}

void setup(void)
{
	vec3d_t location = { 0, 0, 0 };
	vec3d_t direction = normal();
	double d_per_meter = mtodeg(1);

	vec3Scl(direction, direction, d_per_meter * .1); // 10cm

	// setup a fake route. Extending on the y axis
	// for 10 meters, but in degree units
	for(int i = 0; i < ROUTE_LEN; ++i)
	{
		route[i].self.location = location;

		if(i < ROUTE_LEN - 1)
		{
			route[i].next = route + i + 1;
		}

		vec3Add(location, location, direction);		
	}

	// setup an intersecting obstacle
	int_obs.valid = 1;
	int_obs.width = 1; // 1m
	int_obs.radius = d_per_meter;
	vec3Set(int_obs.centroid, route[ROUTE_LEN / 2].self.location);
	int_obs.centroid.x += d_per_meter / 2; // offset a bit

	// copy the intersecting obstacle, but shift it over to the side
	// two widths
	obs = int_obs;
	obs.centroid.x += obs.radius * 2;
}

int test(void)
{
	vec3f_t intersection;
	scn_obstacle_t* int_obs_ptr = &int_obs;
	assert(int_obs_ptr == obs_intersects_route(int_obs_ptr, 1, route, &intersection));
	//assert(obs_intersects_route(&obs, 1, route, &intersection) == NULL);

	return 0;
}

TEST_BEGIN
	.name = "Route obstacle intersection test",
	.description = "Checks to see if a scanner obstacle blocks a route.",
	.run = test,
	.setup = setup,
TEST_END
