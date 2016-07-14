#include "TrackingMat.h"
#include <unistd.h>

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

//------------------------------------------------------------------------------
TrackingMat::~TrackingMat()
{
	for(int i = dimensions.width; i--;){
		free(cols[i]);
	}

	free(cols);
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
void TrackingMat::updateFeatureDeltas(vector<Point2f>* featureList)
{
	// calculate deltas for each feature determine the max
	// delta at the same time
	maxDelta = 1;
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

//------------------------------------------------------------------------------
static void expandBounds(Point2f& min, Point2f& max, Point2f position)
{
	min.x = position.x < min.x ? position.x : min.x;
	min.y = position.y < min.y ? position.y : min.y;
	max.x = position.x > max.x ? position.x : max.x;
	max.y = position.y > max.y ? position.y : max.y;
}
//------------------------------------------------------------------------------
// static int measureRegion(trkRegion_t* region, trkMatFeature_t* feat, int ri, int depth)
// {
// 	int associates = 1;
//
// 	feat->region = -2;
//
// 	if(feat->deltaMag > (TRK_THRESHOLD / feat->bias)){
// 		for(int i = 0; i < TRK_ADJ_FEATURES; ++i){
// 			trkMatFeature_t* adj = feat->adj[i];
// 			if(!adj || adj->region == -2) break; // reached the end
//
// 			// is adj delta close enough to the current delta
// 			float dd = fabs(adj->deltaMag - feat->deltaMag);
// 			if(dd < TRK_COINCIDENCE_THRESHOLD /*&& abs(adj->histBucket - feat->histBucket) < 2*/){
// 				associates += measureRegion(region, adj, ri, depth + 1);
// 			}
// 		}
// 	}
//
// 	feat->region = -1;
//
// 	return associates;
// }
//------------------------------------------------------------------------------
static int measureRegion(trkMatFeature_t* feat, int depth, uint8_t frame)
{
	int associates = 1;

	feat->visited = frame;

	// if(depth < 10)
	if(feat->deltaMag > (TRK_THRESHOLD / feat->bias)){

		// if(depth < 10)
		for(int i = 0; i < TRK_ADJ_FEATURES; ++i){
			trkMatFeature_t* adj = feat->adj[i];
			if(!adj || adj->visited == frame) break; // reached the end

			// is adj delta close enough to the current delta
			float dd = fabs(adj->deltaMag - feat->deltaMag);
			if(dd < TRK_COINCIDENCE_THRESHOLD){
				associates += measureRegion(adj, depth + 1, frame);
			}
		}
	}

	return associates;
}
//------------------------------------------------------------------------------
static int exploreRegion(trkRegion_t* region, Point2f& min, Point2f& max, trkMatFeature_t* feat, int ri, int depth)
{
	int associates = 1;

	if(feat->region > -1) return 0;

	feat->region = ri;
	expandBounds(min, max, feat->position);//Point2f(feat->col, feat->row));

	// if(depth < 10)
	for(int i = 0; i < TRK_ADJ_FEATURES; ++i){
		trkMatFeature_t* adj = feat->adj[i];

		if(!adj || adj->histBucket < 0) break;
		float dd = fabs(adj->deltaMag - feat->deltaMag); // is adj delta close enough to the current delta

		if(dd < TRK_COINCIDENCE_THRESHOLD){
			associates += exploreRegion(region, min, max, adj, ri, depth + 1);
		}
	}

	return associates;
}

//------------------------------------------------------------------------------
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
			regions[i].max = Point2f(-1000, -1000);
			regions[i].min = Point2f(1000, 1000);
	}
	regionCount = 0;

	// reset all the histogram feature lists
	for(int i = TRK_HISTOGRAM_BUCKETS; i--;){
		histFeatureList[i].clear();
	}

	// reset the regions assigned to each feature
	for(int y = dimensions.height; y--;){
		for(int x = dimensions.width; x--;){
			// if(cols[x][y].deltaMag < TRK_THRESHOLD){
				cols[x][y].region = -1;
				// cols[x][y].bias = cols[x][y].bias < 1 ? 1 : cols[x][y].bias * 0.5;
				cols[x][y].histBucket = -1;
			// }
		}
	}

	// group each feature into a velocity magnitude histogram bucket
	for(int y = dimensions.height; y--;){
		for(int x = dimensions.width; x--;){
			if(cols[x][y].deltaMag < TRK_THRESHOLD) continue;
			int8_t bucket = TRK_HISTOGRAM_BUCKETS * (cols[x][y].deltaMag / maxDelta);
			bucket = bucket >= TRK_HISTOGRAM_BUCKETS ? TRK_THRESHOLD - 1 : bucket;
			cols[x][y].histBucket = bucket;
			histFeatureList[bucket].push_back(&(cols[x][y]));
		}
	}

	// iterate over all the feature lists and start walking adjacent
	// features building out regions.
	for(int i = TRK_HISTOGRAM_BUCKETS; i--;){
		for(int j = histFeatureList[i].size(); j--;){
			trkMatFeature_t* feature = histFeatureList[i][j];

			// skip features that are alread claimed by a region.
			if(feature->region >= 0 || feature->deltaMag < TRK_THRESHOLD) continue;
			// if(measureRegion(feature, regionCount, frameCounter) >= TRK_MIN_REGION_SIZE){
			// 	exploreRegion(regions + regionCount, feature, regionCount, 0);
			// 	regionCount++;
			// }
			int size = 0;
			Point2f t_min(1000, 1000);
			Point2f t_max(-1000, -1000);
			trkRegion_t* region = regions + regionCount;
			if((size = exploreRegion(region, t_min, t_max, feature, regionCount, 0)) > TRK_MIN_REGION_SIZE){
				regionCount++;
				expandBounds(region->min, region->max, t_min);
				expandBounds(region->min, region->max, t_max);
				printf("Region size %d\n", size);
			}

			// }
			if(regionCount >= TRK_REGIONS) goto done;
		}

		if(regionCount >= TRK_REGIONS) break;
	}
done: ;

	// find region start points. The region will be expanded/discovered
	// from these points
	// for(int i = TRK_REGIONS; i--;){
	// 	trkRegion_t* region = regions + i;
	// 	if(region->isOutOfBounds(dimensions)){
	// 		region->expPoint = NULL;
	// 	}
	// 	else{
	// 		Point2f c = region->centroid;
	// 		region->expPoint = cols[(int)c.x] + (int)c.y;
	// 	}
	// }
	//
	// for(int i = TRK_REGIONS; i--;){
	// 	trkRegion_t* region = regions + i;
	// 	trkMatFeature_t* ex = region->expPoint;
	// 	if(!ex) continue;
	//
	// 	region->min = region->max = ex->position;//Point2f(ex->col, ex->row);
	// 	int size = exploreRegion(region, ex, i, 0);
	//
	// 	if(1 || size >= TRK_MIN_REGION_SIZE){
	// 		region->flags = TRK_REGION_ACTIVE;
	// 		// region->centroid = (region->min + region->max) / 2;
	// 	}
	// }

	lastFeatureList = featureList;
	frameCounter++;

	return 0;
}
