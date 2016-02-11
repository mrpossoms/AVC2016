#pragma once

#include <opencv2/opencv.hpp>

#define TRK_ADJ_FEATURES 8
#define TRK_REGIONS 16

#define TRK_THRESHOLD 1.0f

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
	int     region;
	float   deltaMag;
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
	Point2i min, max;
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
		trkRegion_t regions[TRK_REGIONS];
		Size2i      dimensions;

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
		float             maxDelta;
		unsigned int      iterations;
};
