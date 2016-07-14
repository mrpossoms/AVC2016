#pragma once

#include <opencv2/opencv.hpp>
#include "types.h"

#define TRK_ADJ_FEATURES      8
#define TRK_REGIONS           256
#define TRK_HISTOGRAM_BUCKETS 16

#define TRK_COINCIDENCE_THRESHOLD 0.25f
#define TRK_THRESHOLD 1.0
//0.75f
// 1.25f

#define TRK_REGION_NONE   0
#define TRK_REGION_ACTIVE 1
#define TRK_MIN_REGION_SIZE 16

using namespace std;
using namespace cv;

//-------------------------------------------------------------------
//    _____
//   |_   _|  _ _ __  ___ ___
//     | || || | '_ \/ -_|_-<
//     |_| \_, | .__/\___/__/
//         |__/|_|

struct MatFeature;
typedef struct MatFeature{
	Point2f position;
	int8_t  col, row;
	int8_t  histBucket; // histogram indexed by vector magnitude
	int8_t  region;     // index of the tracking region this feature belongs to
	int8_t  visited;
	float   deltaMag;
	float   bias;
	Point2f delta;
	struct MatFeature* adj[TRK_ADJ_FEATURES];

	Point2f gradient(){
		float adjCount = 0;
		Point2f grad = Point(0, 0);
		for(int i = 0; i < TRK_ADJ_FEATURES; ++i){
			if(!this->adj[i]) break;
			++adjCount;
			grad += (delta - this->adj[i]->delta);
		}

		return grad / adjCount;
	};
} trkMatFeature_t;

typedef struct{
	int flags;
	int samples;
	Point2f centroid;
	Point2f min, max;
	trkMatFeature_t* expPoint;

	int isOutOfBounds(Size2i dims)
	{
		return isnan(centroid.x * centroid.y) || centroid.x < 0 || centroid.y < 0 ||
				 centroid.x >= dims.width || centroid.y >= dims.height;
	}
} trkRegion_t;

//-------------------------------------------------------------------
//     ___ _
//    / __| |__ _ ______ ___ ___
//   | (__| / _` (_-<_-</ -_|_-<
//    \___|_\__,_/__/__/\___/__/
//
class TrackingMat{
	public:
		int         regionCount;
		int8_t      frameCounter;
		Size2i      dimensions;
		trkRegion_t regions[TRK_REGIONS];

		TrackingMat(Size2i size);
		~TrackingMat();
		int update(vector<Point2f>* featureList);

		trkMatFeature_t* operator[](const int x)
		{
			return this->cols[x];
		}

	private:
		trkMatFeature_t** cols;
		vector<Point2f>*  lastFeatureList;
		vector<trkMatFeature_t*> histFeatureList[TRK_HISTOGRAM_BUCKETS];
		float             maxDelta;
		unsigned int      iterations;

		void assignRegions();
		void updateFeatureDeltas(vector<Point2f>* featureList);
		trkRegion_t* nearestCentroid(int col, int row, float* dist, vec3f_t* delta);
};
