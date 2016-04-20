#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <iostream>
#include <cmath>
#include <chrono>
#include <unistd.h>

#include "Aria.h"


class KalmanFilter
{
public:
  double X = 0.0;
  double Y = 0.0;
  double Phi = 0.0;
  
  KalmanFilter(ArRobot* robot);
  void propagate(double dt);
  
private:
  ArRobot* robot;
};

#endif // KALMANFILTER_H
