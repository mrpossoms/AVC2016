#include "test.h"
#include "sensors/gps.h"
#include "sensors/scanner.c"

#define ROUTE_LEN 100

#define RAND_F (random() % 2048 / 1024.f)

gpsWaypointCont_t route[ROUTE_LEN];
scn_obstacle_t int_obs, between_obs, obs;

vec3d_t normal()
{
	vec3d_t v = { RAND_F, RAND_F, RAND_F };
	double mag = sqrt(vec3Dot(v, v));
	vec3Scl(v, v, 1 / mag);
	return v;
}

vec3d_t up()
{
	vec3d_t v = { 0, 1, 0 };
	return v;
}

void setup(void)
{
	vec3d_t location = { 0, 0, 0 };
	vec3d_t direction = normal();
	double d_per_meter = mtodeg(1);

	vec3Scl(direction, direction, d_per_meter);
	assert(sqrt(vec3Dot(direction, direction)) == d_per_meter);

	//printf("direction --(%e, %e)-->\n", direction.x, direction.y);

	// setup a fake route. Extending on the y axis
	// for 100 meters, but in degree units
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
	vec3Set(int_obs.centroid, route[19].self.location);
	int_obs.centroid.x += d_per_meter / 2; // offset a bit

	// copy the intersecting obstacle, but shift it over to the side
	// two widths
	obs = int_obs;
	obs.centroid.x += obs.radius * 2;

	// place obstacle between two waypoints to check for intersection correctness
	between_obs.valid = 1;
	between_obs.width = 0.5; // 5cm
	between_obs.radius = d_per_meter * 0.5; // 2.5cm radius
	vec3Add(between_obs.centroid, route[0].self.location, route[1].self.location);
	vec3Scl(between_obs.centroid, between_obs.centroid, 0.5); // place betwen 0 & 1

/*
	printf("(%e, %e) -- (%e, %e) -- (%e, %e)\n", 
		route[0].self.location.x, route[0].self.location.y,
		between_obs.centroid.x, between_obs.centroid.y,
		route[1].self.location.x, route[1].self.location.y
	);
*/
}

int test(void)
{
	vec3f_t intersection;
	gpsWaypointCont_t* before_intersect = NULL;
	scn_obstacle_t* int_obs_ptr = &int_obs;
	scn_obstacle_t* bet_obs_ptr = &between_obs;
	assert(int_obs_ptr == obs_intersects_route(int_obs_ptr, 1, route, &before_intersect));
	assert(bet_obs_ptr == obs_intersects_route(bet_obs_ptr, 1, route, &before_intersect));
	assert(obs_intersects_route(&obs, 1, route, &before_intersect) == NULL);

	return 0;
}

TEST_BEGIN
	.name = "Route obstacle intersection test",
	.description = "Checks to see if a scanner obstacle blocks a route.",
	.run = test,
	.setup = setup,
TEST_END
