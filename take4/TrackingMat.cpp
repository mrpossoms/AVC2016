#include "TrackingMat.h"

TrackingMat::TrackingMat(Size2i size)
{
	dimensions = size;

	// allocate all the elements
	cols = (trkMatFeature_t**)malloc(sizeof(trkMatFeature_t*) * size.width);
	for(int i = size.width; i--;){
		cols[i] = (trkMatFeature_t*)malloc(sizeof(trkMatFeature_t) * size.height);
	}



	// stitch together all the adjacent features
	const int adjOffsets[][2] = {
		{ -1, -1 }, // NW
		{  0, -1 }, // N
		{  1, -1 }, // NE
		{  1,  0 }, // E
		{  1,  1 }, // SE
		{  0,  1 }, // S
		{ -1,  1 }, // SW
		{ -1,  0 }, // W
	};
	for(int y = 0; y < size.height; ++y)
		for(int x = 0; x < size.width; ++x){
			// stitch here
			int ai = 0;
			for(int i = TRK_ADJ_FEATURES; i--;){
				int offx = x + adjOffsets[i][0], offy = y + adjOffsets[i][1];
				if(offx < 0 || offy < 0 || offx >= size.width || offy >= size.height){
					continue;
				}
				else{				
					cols[x][y].adj[ai++] = &(cols[offx][offy]);
				}
			}
		}
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
	if(!this->lastFeatureList){
		this->lastFeatureList = featureList;
		return -1;
	}

	// calculate deltas for each feature determine the max
	// delta at the same time
	maxDelta = 0;
	for(int i = featureList->size(); i--;){
		int x = i % dimensions.width;
		int y = i / dimensions.height;

		Point2f delta = cols[x][y].delta = (*this->lastFeatureList)[i] - (*featureList)[i];
		float mag = sqrtf(delta.x * delta.x + delta.y * delta.y);
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
			int   smallestAri = 0;
			for(int i = 0; i < TRK_ADJ_FEATURES; ++i){
				trkMatFeature_t* adj = feature->adj[i];
				if(!adj) break; // reached the end
				int ari = TRK_REGIONS * (adj->deltaMag / maxDelta);

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

	this->lastFeatureList = featureList;

	return 0;
}