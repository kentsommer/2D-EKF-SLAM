#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include "Aria.h"

#include "houghtransform.h"

typedef struct Feature{
  double x, y;
} Feature;


class FeatureDetector
{
public:
  double NO_COMPASS = 100.0;
  
  static const int MAX_DIST = HoughTransform::MAX_DIST;
  static const int MIN_POINTS = 3;  //minimum number of points needed for a line segment
  static const int POINT_DIST = 600; //distance between points on same line segment (mm)
  
  double CORNER_THETA = 22.0 * 3.141592654/180.0;   //min angle between segments making a corner
  static const int CORNER_DIST = 90000; //squared distance between segment and feature (mm)
  
  FeatureDetector(ArSick* sick);
  ~FeatureDetector();
  
  int getFeatures(std::vector<Feature> *featVec, double* structCompass);
  
private:
  struct lineSegment{
    double radius;
    double theta;
    double startX, startY;
    double endX, endY;
    int numPoints;
    struct lineSegment* next;
  };

  ArSick* sick;
  HoughTransform* hough;
  
  int fitLineSegments(std::vector<ArSensorReading> *readings, 
                      std::vector<struct houghLine> *lines, 
                      std::vector<struct lineSegment> *segments);
  
  struct lineSegment* mergeSegmentsX(struct lineSegment* segments);
  struct lineSegment* mergeSegmentsY(struct lineSegment* segments);
    
  double getStructCompass(std::vector<struct lineSegment> *segments);
  int extractCorners(std::vector<Feature> *featVec, std::vector<struct lineSegment> *segments);
};

#endif // FEATUREDETECTOR_H
