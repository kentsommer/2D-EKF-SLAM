#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(ArRobot* robot){
  this->robot = robot;
}


void KalmanFilter::propagate(double dt) {
  robot->lock();
    double V = robot->getVel();
    double RTV = robot->getRotVel() * 3.141592654 / 180.0;
  robot->unlock();
  
  X = X + dt*V*cos(Phi);
  Y = Y + dt*V*sin(Phi);
  Phi = Phi + dt*RTV;
  
  if (Phi > 2*3.141592654) Phi -= (3.141592654*2);
  if (Phi < 0.0) Phi += (3.141592654*2);

  usleep(1000);
}