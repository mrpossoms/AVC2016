#include "TrackingMat.h"

#define FEATURE_COUNT  (dimensions.width * dimensions.height)

TrackingMat::TrackingMat(Size2i size)
{
	// initialize
	dimensions = size;
	lastFeatureList = NULL;
	regionCount = 0;
	iterations  = 0;

	// allocate all the elements
	size_t allocated = sizeof(trkMatFeature_t*) * size.width;
	cols = (trkMatFeature_t**)malloc(sizeof(trkMatFeature_t*) * size.width);
	for(int i = size.width; i--;){
		size_t colBytes = sizeof(trkMatFeature_t) * size.height;
		allocated += colBytes;
		cols[i] = (trkMatFeature_t*)malloc(colBytes);
	}

	printf("%zuB allocated\n", allocated);
	sleep(1);

	// stitch together all the adjacent features
	const int adjOffsets[TRK_ADJ_FEATURES][2] = {
		{ -1, -1 }, // NW
		{  0, -1 }, // N
		{  1, -1 }, // NE
		{  1,  0 }, // E
		{  1,  1 }, // SE
		{  0,  1 }, // S
		{ -1,  1 }, // SW
		{ -1,  0 }, // W
	};

	printf("=============\nSETUP\n=============\n");
	for(int y = 0; y < size.height; ++y)
		for(int x = 0; x < size.width; ++x){
			bzero(cols[x][y].adj, sizeof(cols[x][y].adj));
			cols[x][y].region = -1;
			cols[x][y].bias = 1;
			cols[x][y].histBucket = -1;
			cols[x][y].col = x;
			cols[x][y].row = y;
			// stitch here
			int ai = 0;
			for(int i = 0, j = 0; i < TRK_ADJ_FEATURES; ++i){
				int offx = x + adjOffsets[i][0], offy = y + adjOffsets[i][1];

				if(offx >= 0 && offy >= 0 && offx < size.width && offy < size.height){
						cols[x][y].adj[j++] = cols[offx] + offy;
				}

				// printf("S %d (%d, %d) adj[%d] = %x\n", y * dimensions.width + x, x, y, i, (void*)cols[x][y].adj[i]);
			}
		}

		// distribute the inital tracking region centroids
		for(int i = TRK_REGIONS; i--;){
			regions[i].centroid = Point2f(
				random() % size.width,
				random() % size.height
			);
			printf("ri %d (%0.3f, %0.3f)\n", i, regions[i].centroid.x, regions[i].centroid.y);
		}
		printf("=============\nFINISHED\n=============\n");
}

TrackingMat::~TrackingMat()
{
	for(int i = dimensions.width; i--;){
		free(cols[i]);
	}

	free(cols);
}

static int mostCommon(trkMatFeature_t* adj[], int adjCount)
{
	int median = -1;
	int occurances = 0;

	for(int i = adjCount; i--;){
		int ri = adj[i]->region;
		int ric = 0;

		for(int j = adjCount; j--;){
			if(adj[j]->region == ri && adj[j]->region > -1){
				++ric;
			}
		}

		if(ric > occurances){
			median = ri;
			occurances = ric;
		}
	}

	return median;
}

trkRegion_t* TrackingMat::nearestCentroid(int col, int row, float* distSqr, vec3f_t* delta)
{
	assert(col >= 0 && row >= 0);

	trkRegion_t* closest = NULL;
	vec3f_t closestDelta;
	float   closestDistSqr;

	int ni = 0;
	for(int i = TRK_REGIONS; i--;){
		delta->x = col - regions[i].centroid.x;
		delta->y = row - regions[i].centroid.y;
		float d = vec3fDot(delta, delta);
		if(d < closestDistSqr || !closest){
			closestDistSqr = d;
			closest = regions + (ni = i);
			closestDelta = *delta;
		}
	}

	// printf("%d (%f, %f) nearest to (%d, %d)\n", ni, closest->centroid.x, closest->centroid.y, col, row);
	*delta   = closestDelta;
	*distSqr = closestDistSqr;

	return closest;
}

void TrackingMat::updateFeatureDeltas(vector<Point2f>* featureList)
{
	// calculate deltas for each feature determine the max
	// delta at the same time
	maxDelta = 0;
	for(int i = lastFeatureList->size(); i--;){
		int x = i % dimensions.width;
		int y = i / dimensions.height;

		assert(lastFeatureList != featureList);
		// assert(lastFeatureList && featureList);

		// the new feature list is shorter than the last one
		// quit before we get into trouble
		if(i >= featureList->size()) break;

		Point2f delta = (*lastFeatureList)[i] - (*featureList)[i];
		float mag = sqrtf(delta.x * delta.x + delta.y * delta.y);

		// printf("U %d (%d, %d) = %x\n", i, x, y, (void*)(cols[x] + y));
		cols[x][y].delta = delta;
		cols[x][y].deltaMag = mag;
		cols[x][y].position = (*featureList)[i];

		maxDelta = mag > maxDelta ? mag : maxDelta;
	}
}

static void expandBounds(trkRegion_t* region, Point2f position)
{
	region->min.x = position.x < region->min.x ? position.x : region->min.x;
	region->min.y = position.y < region->min.y ? position.y : region->min.y;
	region->max.x = position.x > region->max.x ? position.x : region->max.x;
	region->max.y = position.y > region->max.y ? position.y : region->max.y;
}

static int expandRegion(trkRegion_t* region, trkMatFeature_t* feat, int ri, int depth)
{
	int associates = 1;

	if(feat->deltaMag > (TRK_THRESHOLD / feat->bias)){
		feat->region = ri;
		feat->bias++;
		expandBounds(region, feat->position);//Point2f(feat->col, feat->row));

		for(int i = 0; i < TRK_ADJ_FEATURES; ++i){
			trkMatFeature_t* adj = feat->adj[i];
			if(!adj || adj->region > -1) break; // reached the end

			// is adj delta close enough to the current delta
			float dd = fabs(adj->deltaMag - feat->deltaMag);
			if(dd < TRK_COINCIDENCE_THRESHOLD /*&& abs(adj->histBucket - feat->histBucket) < 2*/){
				associates += expandRegion(region, adj, ri, depth + 1);
			}
		}
	}

	return associates;
}

int TrackingMat::update(vector<Point2f>* featureList)
{
	if(!lastFeatureList){
		lastFeatureList = featureList;
		return -1;
	}

	assert(featureList->size() <= FEATURE_COUNT);

	updateFeatureDeltas(featureList);

	// reset the region markers for each feature
	for(int i = 0; i < TRK_REGIONS; ++i){
			regions[i].flags = TRK_REGION_NONE;
			regions[i].samples >>= 1;
	}
	regionCount = 0;

	// reset the regions assigned to each feature
	for(int y = dimensions.height; y--;){
		for(int x = dimensions.width; x--;){
			if(cols[x][y].deltaMag < TRK_THRESHOLD){
				cols[x][y].region = -1;
				cols[x][y].bias = cols[x][y].bias < 1 ? 1 : cols[x][y].bias * 0.5;
			}
		}
	}

	// group each feature into a velocity magnitude histogram bucket
	for(int y = dimensions.height; y--;){
		for(int x = dimensions.width; x--;){
			if(cols[x][y].deltaMag < TRK_THRESHOLD) continue;

			cols[x][y].histBucket = TRK_HISTOGRAM_BUCKETS * (cols[x][y].deltaMag / maxDelta);

			float dist = 0;
			vec3f_t delta = {};
			trkRegion_t* region = nearestCentroid(x, y, &dist, &delta);

			if(!region) continue;

			float weight = 1.0f / (sqrt(dist) + region->samples);
			delta = vec3fScl(&delta, weight);

			// assert(region->centroid.x >= 0 && region->centroid.y >= 0);
			region->centroid += Point2f(delta.x, delta.y);
			// printf("delta (%f, %f)\n", delta.x, delta.y);
			// assert(region->centroid.x >= 0 && region->centroid.y >= 0);
			region->samples++;

			// region->mass += weight;
		}
	}

	// find region start points. The region will be expanded/discovered
	// from these points
	for(int i = TRK_REGIONS; i--;){
		trkRegion_t* region = regions + i;
		if(region->isOutOfBounds(dimensions)){
			region->expPoint = NULL;
		}
		else{
			Point2f c = region->centroid;
			region->expPoint = cols[(int)c.x] + (int)c.y;
		}
	}

	for(int i = TRK_REGIONS; i--;){
		trkRegion_t* region = regions + i;
		trkMatFeature_t* ex = region->expPoint;
		if(!ex) continue;

		region->min = region->max = ex->position;//Point2f(ex->col, ex->row);
		int size = expandRegion(region, ex, i, 0);

		if(1 || size >= TRK_MIN_REGION_SIZE){
			region->flags = TRK_REGION_ACTIVE;
			// region->centroid = (region->min + region->max) / 2;
		}
	}

	lastFeatureList = featureList;

	return 0;
}
