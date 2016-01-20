#include "TrackingMat.h"

TrackingMat::TrackingMat(Size2i size)
{
	dimensions = size;

	// allocate all the elements
	cols = (matFeature_t**)malloc(sizeof(matFeature_t*) * size.width);
	for(int i = size.width; i--;){
		cols[i] = (matFeature_t*)malloc(sizeof(matFeature_t) * size.height);
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
			for(int i = ADJ_FEATURES; i--;){
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

		Point2f delta = (*this->lastFeatureList)[i] - (*featureList)[i];
		float mag = sqrtf(delta.x * delta.x + delta.y * delta.y);
		cols[x][y].delta = mag;

		maxDelta = mag > maxDelta ? mag : maxDelta;
	}

	regionCount = 0;
	for(int y = dimensions.height; y--;){
		for(int x = dimensions.width; x--;){
			cols[x][y].region = -1;
		}
	}
	for(int y = dimensions.height; y--;){
		for(int x = dimensions.width; x--;){
			matFeature_t* feature = cols[x] + y;
			int ri = REGIONS * (feature->delta / maxDelta);
			float delta = feature->delta;


			float smallestDelta = 1000000;
			int   smallestAri = 0;
			for(int i = 0; i < ADJ_FEATURES; ++i){
				matFeature_t* adj = feature->adj[i];
				if(!adj) break; // reached the end
				int ari = REGIONS * (adj->delta / maxDelta);

				// is adj delta close enough to the current delta
				float dd = powf(adj->delta - delta, 2.0f);
				if(dd < TRACKING_THRESHOLD){
					if(adj->region > -1){
						feature->region = adj->region;
					}
					else if(dd <= smallestDelta){
						smallestDelta = dd;
						smallestAri   = ari;
					}
				}
			}
		}
	}

	this->lastFeatureList = featureList;

	return 0;
}