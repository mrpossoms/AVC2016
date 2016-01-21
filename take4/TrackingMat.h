#pragma once

#include <opencv2/opencv.hpp>

#define TRK_ADJ_FEATURES 8
#define TRK_REGIONS 16

#define TRK_THRESHOLD 1.0f

using namespace std;
using namespace cv;

struct MatFeature;
typedef struct MatFeature{
	Point2f position;
	int     region;
	float   deltaMag;
	Point2f delta;
	struct MatFeature* adj[TRK_ADJ_FEATURES];
} trkMatFeature_t;

typedef struct{
	Point2i min, max;
} trkRegion_t;

class TrackingMat{
	public:
		int regionCount;
		trkRegion_t regions[TRK_REGIONS];

		TrackingMat(Size2i size);
		~TrackingMat();
		int update(vector<Point2f>* featureList);

		trkMatFeature_t* operator[](const int x)
		{
			return this->cols[x];
		}

	private:
		Size2i           dimensions;
		trkMatFeature_t**   cols;
		vector<Point2f>* lastFeatureList;
		float            maxDelta;
};