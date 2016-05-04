#include "kalmanfilter.h"
#include <math.h>

KalmanFilter::KalmanFilter(ArRobot* robot){
  this->robot = robot;
  
  state = new Eigen::VectorXd(3);
  covariance = new Eigen::MatrixXd(3,3);
  
  (*state) << 0.0, 0.0, 0.0;
  (*covariance) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}


void KalmanFilter::doPropagation(double dt) {
  // std::cout << "\nX_hat_min =" << std::endl;
  // std::cout << (*state) << std::endl << std::endl;
  //std::cout << "P_min =" << std::endl;
  //std::cout << (*covariance) << std::endl;
  
  
  robot->lock();
    double V = robot->getVel();
    double RTV = robot->getRotVel() * 3.141592654 / 180.0;
  robot->unlock();
  
  
  int n = (*state).size();          //size of state vector, #of Landmark = (n-3)/2

  double v =V/1000.0;
  double w = RTV;
  double sigma_v = 0.01;
  double sigma_w = 0.04;
  
  Eigen::MatrixXd Q(2,2);        //Covariance of noise of Control input: linear/rotational velocity
  
  Eigen::MatrixXd Set;
  
  // Get the result of Propagation
  Q << sigma_v, 0,
      0, sigma_w;
  Q = (v*v)*Q*Q;
//   Set = Propagate(x_hat_min, m, 0.1, 0.01, Q, 1);
  Set = Propagate(*state, *covariance, v, w, Q, dt);
  
  // and parse them out to state vector and covariance matrix
  (*state) = Set.block(0,0,n,1);
  (*covariance) = Set.block(0,1,n,n);
  
  X = (*state)(0);
  Y = (*state)(1);
  Phi = (*state)(2);
  
  
  
  //printf("%f\n", sin(pi/2));
}

void KalmanFilter::doUpdate(Eigen::MatrixXd z_chunk, Eigen::MatrixXd R_chunk) {

  //Set Mahalanobis thresholds
  int Gamma_max = 300;
  int Gamma_min = 4;
  //Variable to hold out size
  int size;
  // Hold out matrices state and covariance
  Eigen::MatrixXd Set;

  //Set = Update1(*state, *covariance, z_chunk, R_chunk, Gamma_max, Gamma_min);
  Set = Update2(*state, *covariance, z_chunk, R_chunk, Gamma_max, Gamma_min);
  
  size = sqrt(Set.size());
  
  delete state;
  delete covariance;
  state = new Eigen::VectorXd(size);
  covariance = new Eigen::MatrixXd(size,size);
  
  (*state) = Set.block(0, 0, size, 1);
  (*covariance) = Set.block(0, 1, size, size);
  
  X = (*state)(0);
  Y = (*state)(1);
  Phi = (*state)(2);


}