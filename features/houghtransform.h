#ifndef HOUGHTRANSFORM_H
#define HOUGHTRANSFORM_H

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include "Aria.h"

struct houghLine{
  double radius;
  double theta;
  double weight;
};


class HoughTransform
{
public:
  static const int MAX_DIST = 5000;               //maximum distance (mm) to lines
  static const int DISTANCE = 10;                 //distance between lines detected (mm)
  static const int THETA_SIZE = 180;              //number of bins to use for 180 degrees
  static const int RADIUS_SIZE = 2*MAX_DIST/DISTANCE + 1;  //Hough grid size
  static const int ADDITION = RADIUS_SIZE/2;      //used to make radii positive
  
  static const int NUM_PEAKS = 85;               //number of peaks used to make lines
  static const int MERGE_THETA = 30;              //angular-distance between peaks belonging to the same line
  static const int MERGE_RADIUS = 5;              //radial-distance between peaks belonging to the same line
  static const int MIN_PEAKS = 3;                 //minimum number of peaks required for a line
  
  HoughTransform();
  ~HoughTransform();
  
  int getLines(std::vector<ArSensorReading> *readings, std::vector<struct houghLine> *lines);
  void clearHoughGrid();
  void sendHoughToImage(char* filename);
  
private:
  struct peakGroup{
    int maxRadius;
    int minRadius;
    int maxTheta;
    int minTheta;
    int radius;
    int theta;
    int weight;
    int numPoints;
  };
  
  float D_THETA;
  float* COS_ARRAY;
  float* SIN_ARRAY;
  unsigned char* houghGrid = nullptr;
  
  void performHoughTransform(std::vector<ArSensorReading> *readings);
  
  void getPeaks(int count, int peaks[]);
};

#endif // HOUGHTRANSFORM_H
