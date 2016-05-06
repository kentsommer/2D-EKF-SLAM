#include "kalmanfilter.h"
#include <math.h>

KalmanFilter::KalmanFilter(ArRobot* robot){
  this->robot = robot;
  
  state = new Eigen::VectorXd(3);
  covariance = new Eigen::MatrixXd(3,3);
  
  (*state) << 0.0, 0.0, 0.0;
  (*covariance) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}


void KalmanFilter::doPropagation(double dt, std::ofstream& file) {
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
  double w = RTV + 0.01; //Correct for that awesome *joke* turning that the robot can't sense
  double sigma_v = 0.01;
  double sigma_w = 0.04;
  
  Eigen::MatrixXd Q(2,2);        //Covariance of noise of Control input: linear/rotational velocity
  
  Eigen::MatrixXd Set;
  
  // Get the result of Propagation
  Q << sigma_v, 0,
      0, sigma_w;
  Q = (v*v)*Q*Q;

    //TIMER
/*  std::clock_t    start;
  start = std::clock();*/

//   Set = Propagate(x_hat_min, m, 0.1, 0.01, Q, 1);
  Set = Propagate(*state, *covariance, v, w, Q, dt);

  //TIMER
  //std::cout << "Time for propagate: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  
  // and parse them out to state vector and covariance matrix
  (*state) = Set.block(0,0,n,1);
  (*covariance) = Set.block(0,1,n,n);
  
  X = (*state)(0);
  Y = (*state)(1);
  Phi = (*state)(2);
  
  file << Set(0,1) << " " << Set(0,2) << " " << Set(1,1) << " " << Set(1,2) << std::endl;
  //std::cout << Set(0,0) << " " << Set(1,1) << std::endl;
  //std::cout << Set(0,1) << " " << Set(0,2) << " " << Set(1,1) << " " << Set(1,2) << std::endl;
  
  //printf("%f\n", sin(pi/2));
}

void KalmanFilter::doUpdate(Eigen::MatrixXd z_chunk, Eigen::MatrixXd R_chunk) {

  //Set Mahalanobis thresholds
  int Gamma_max = 90;
  int Gamma_min = 1;
  //Variable to hold out size
  int size;
  // Hold out matrices state and covariance
  Eigen::MatrixXd Set;

  //TIMER
/*  std::clock_t    start;
  start = std::clock();*/

  //Set = Update1(*state, *covariance, z_chunk, R_chunk, Gamma_max, Gamma_min);
  //Set = Update4(*state, *covariance, z_chunk, R_chunk, Gamma_max, Gamma_min);
  Set = Update3(*state, *covariance, z_chunk, R_chunk, Gamma_max, Gamma_min);

  //TIMER
  //std::cout << "Time for update: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

  
  size = sqrt(Set.size());
  
  delete state;
  delete covariance;
  delete knownLandmarks;
  state = new Eigen::VectorXd(size);
  covariance = new Eigen::MatrixXd(size,size);
  if(size > 3){
    knownLandmarks = new Eigen::MatrixXd((size-3)/2, 2);
  }
  
//   std::cout << "size is: " << size << std::endl;
//   std::cout << "starting from index 3, going up " << size-3 << " indexes." << std::endl;

  (*state) = Set.block(0, 0, size, 1);

  (*covariance) = Set.block(0, 1, size, size);

/*  for (int i=0; i<(size-3)/2; i++){

  }
  (*knownLandmarks) = Set.block(0,2,size-3,1);*/
  
  X = (*state)(0);
  Y = (*state)(1);
  Phi = (*state)(2);


}