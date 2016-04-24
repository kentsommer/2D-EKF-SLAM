#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <iostream>
#include <cmath>
#include <chrono>
#include <unistd.h>
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <stdio.h>

#include "Aria.h"


class KalmanFilter
{
public:
  double X = 0.0;
  double Y = 0.0;
  double Phi = 0.0;
  
  KalmanFilter(ArRobot* robot);
  void doPropagation(double dt);
  
private:
  ArRobot* robot;
  
  Eigen::VectorXd* state;
  Eigen::MatrixXd* covariance;
  
  Eigen::MatrixXd Propagate(Eigen::VectorXd x_hat_plus, Eigen::MatrixXd P_plus, double v_m, double w_m, Eigen::MatrixXd Q, double dt);
};

#endif // KALMANFILTER_H
