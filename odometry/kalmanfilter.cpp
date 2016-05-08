#include "kalmanfilter.h"
#include <math.h>

KalmanFilter::KalmanFilter(ArRobot* robot){
  this->robot = robot;
  
  state = new Eigen::VectorXd(3);
  covariance = new Eigen::MatrixXd(3,3);
  
  (*state) << 0.0, 0.0, 0.0;
  (*covariance) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}


void KalmanFilter::doPropagation(double dt, std::ofstream& covFile) {
  
  robot->lock();
    double V = robot->getVel();
    double RTV = robot->getRotVel() * 3.141592654 / 180.0;
  robot->unlock();
  
  //size of state vector, #of Landmark = (n-3)/2
  int n = (*state).size();

  //Correct for that awesome *joke* turning that the robot can't sense
  double v = V/1000.0;
  double w = RTV + 0.01;
  double sigma_v = 0.01;
  double sigma_w = 0.04;
  
  //Covariance of noise of Control input: linear/rotational velocity
  Eigen::MatrixXd Q(2,2);
  Eigen::MatrixXd Set;
  
  Q << sigma_v, 0,
      0, sigma_w;
  Q = (v*v)*Q*Q;

  // Get the result of Propagation
  Set = Propagate(*state, *covariance, v, w, Q, dt);
  
  // and parse them out to state vector and covariance matrix
  (*state) = Set.block(0,0,n,1);
  (*covariance) = Set.block(0,1,n,n);
  
  X = (*state)(0);
  Y = (*state)(1);
  Phi = (*state)(2);

  //Write robot covariance to file
  covFile << Set(0,1) << " " << Set(0,2) << " " << Set(1,1) << " " << Set(1,2) << std::endl;
}

void KalmanFilter::doUpdate(Eigen::MatrixXd z_chunk, Eigen::MatrixXd R_chunk) {

  //Set Mahalanobis thresholds
  int Gamma_max = 50;
  int Gamma_min = 10;
  //Variable to hold out size
  int size;
  // Hold out matrices state and covariance
  Eigen::MatrixXd Set;

  Set = Update(*state, *covariance, z_chunk, R_chunk, Gamma_max, Gamma_min);
  
  size = sqrt(Set.size());
  
  delete state;
  delete covariance;
  state = new Eigen::VectorXd(size);
  covariance = new Eigen::MatrixXd(size,size);

  (*state) = Set.block(0, 0, size, 1);
  (*covariance) = Set.block(0, 1, size, size);
  Num_Landmarks = ((*state).size() - 3) / 2;
  
  X = (*state)(0);
  Y = (*state)(1);
  Phi = (*state)(2);
}




//run an update with structual compass
void KalmanFilter::doUpdateCompass(double z, double R){
  //get current direction mod 360 degrees as measurement estimate
  double z_hat = (*state)(2);
  z_hat -= 6.283185307 * floor(z_hat / 6.283185307);
  
  //get all possible residuals (accounting for 0 -> 360 degree rollovers (the bane of my existance!))
  double res1 = z - z_hat;
  double res2 = z - 6.283185307 - z_hat;
  double res3 = z + 6.283185307 - z_hat;
  
  //get lowest residuals among all possible ones (assume we're close to measured value)
  double res;
  if ((fabs(res1) <= fabs(res2)) && (fabs(res1) <= fabs(res3))) res = res1;
  else if (fabs(res2) <= fabs(res3)) res = res2;
  else res = res3;
  
  //H matrix is just [1] for this measurement, so H * S * H_transpose is just the
    //covariance of the robot's angle with itself
  double S = (*covariance)(2,2) + R;
  
  //P * H_transpose just pulls out the phi column, calculate K from this
  int size = (*state).size();
  Eigen::MatrixXd K = (1/S) * (*covariance).block(0,2,size,1);
  
  //update state and covariance
  (*state) = (*state) + (res * K);
  (*covariance) = (*covariance) - S * K * K.transpose();
  Eigen::MatrixXd tempTrans = 0.5 * ((*covariance) + (*covariance).transpose());
  (*covariance) = tempTrans;
  
  //save state vals for easy lookup outside of function
  X = (*state)(0);
  Y = (*state)(1);
  Phi = (*state)(2);
}










