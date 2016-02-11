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

			// stitch here
			int ai = 0;
			for(int i = TRK_ADJ_FEATURES; i--;){
				int offx = x + adjOffsets[i][0], offy = y + adjOffsets[i][1];
				if(offx < 0 || offy < 0 || offx >= size.width || offy >= size.height){
					cols[x][y].adj[i] = NULL;
					// continue;
				}
				else{
					cols[x][y].adj[i] = cols[offx] + offy;
				}

				printf("S %d (%d, %d) adj[%d] = %x\n", y * dimensions.width + x, x, y, i, (void*)cols[x][y].adj[i]);
			}
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

int TrackingMat::update(vector<Point2f>* featureList)
{
	if(!lastFeatureList){
		lastFeatureList = featureList;
		return -1;
	}

	assert(featureList->size() <= FEATURE_COUNT);

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

	// reset the region markers for each feature
	regionCount = 0;
	for(int y = dimensions.height; y--;){
		for(int x = dimensions.width; x--;){
			cols[x][y].region = -1;
		}
	}

	// assign regions to each feature
	for(int y = dimensions.height; y--;){
		for(int x = dimensions.width; x--;){
			trkMatFeature_t* feature = cols[x] + y;
			int ri = TRK_REGIONS * (feature->deltaMag / maxDelta);
			float delta = feature->deltaMag;

			float smallestDelta = 1000000;
			int   smallestAri = 0; // adjacent region index?

			// check all features adjacent to 'feature'
			// first check to see how closely their movement matches
			// 'feature's movement. If there are tolerable, check to
			// see if they have been assigned a region, if not use the
			// region that will, so far, be assigned to 'feature'
			for(int i = 0; i < TRK_ADJ_FEATURES; ++i){
				trkMatFeature_t* adj = feature->adj[i];
				if(!adj) break; // reached the end
				int ari = 0;//TRK_REGIONS * (adj->deltaMag / maxDelta);

				// is adj delta close enough to the current delta
				float dd = powf(adj->deltaMag - delta, 2.0f);
				if(dd < TRK_THRESHOLD){
					// has a region already been assigned?
					if(adj->region > -1){
						feature->region = adj->region;
					}
					// region is unassigned, but it's the closest so far
					// keep track of it so we can assign a new region
					else if(dd <= smallestDelta){
						smallestDelta = dd;
						smallestAri   = ari;
					}
				}
			}

			if(smallestAri > regionCount){
				regionCount = smallestAri;
			}
		}
	}

	lastFeatureList = featureList;

	return 0;
}
