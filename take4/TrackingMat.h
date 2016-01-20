#pragma once

#include <opencv2/opencv.hpp>

#define ADJ_FEATURES 8
#define REGIONS 16

#define TRACKING_THRESHOLD 1.0f

using namespace std;
using namespace cv;

struct MatFeature;
typedef struct MatFeature{
	int    region;
	float  delta;
	struct MatFeature* adj[ADJ_FEATURES];
} matFeature_t;

typedef struct{
	Point2i min, max;
} region_t;

class TrackingMat{
	public:
		int regionCount;
		region_t regions[REGIONS];

		TrackingMat(Size2i size);
		~TrackingMat();
		int update(vector<Point2f>* featureList);

		matFeature_t* operator[](const int x)
		{
			return this->cols[x];
		}

	private:
		Size2i           dimensions;
		matFeature_t**   cols;
		vector<Point2f>* lastFeatureList;
		float            maxDelta;
};