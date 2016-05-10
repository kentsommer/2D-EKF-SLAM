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
#include <ctime>
#include <fstream>

#include "Aria.h"

#define INF 999999999999
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286


class KalmanFilter
{
public:
  double X = 0.0;
  double Y = 0.0;
  double Phi = 0.0;
  int Num_Landmarks = 0;
  int Prev_Landmarks = 0;
  Eigen::VectorXd* landmarks;
  Eigen::MatrixXd* knownLandmarks;
  
  KalmanFilter(ArRobot* robot);
  void doPropagation(double dt, std::ofstream& covFile, std::ofstream& knownfeaturesFile);
  void doUpdate(Eigen::MatrixXd z_chunk, Eigen::MatrixXd R_chunk);
  void doUpdateCompass(double z, double R);
  
private:
  ArRobot* robot;
  
  Eigen::VectorXd* state;
  Eigen::MatrixXd* covariance;
  
  Eigen::MatrixXd Propagate(Eigen::VectorXd x_hat_plus, Eigen::MatrixXd P_plus, double v_m, double w_m, Eigen::MatrixXd Q, double dt);

  Eigen::MatrixXd Update(Eigen::VectorXd x_hat_min, Eigen::MatrixXd P_min, Eigen::MatrixXd z_chunk, Eigen::MatrixXd R_chunk, int Gamma_max, int Gamma_min);
};

#endif // KALMANFILTER_H
