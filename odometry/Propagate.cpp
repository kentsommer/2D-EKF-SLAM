#include "kalmanfilter.h"

/* 

Return a matrix composed of following:

   // 1st column: x_hat_min

   // 2nd column to last column: P_min

   These are then parsed in the main function		
	
	*/ 

Eigen::MatrixXd KalmanFilter::Propagate(Eigen::VectorXd x_hat_plus, Eigen::MatrixXd P_plus, double v_m, double w_m, Eigen::MatrixXd Q, double dt)
{
	int n = x_hat_plus.size();
	
	double ori = x_hat_plus(2);

		
	Eigen::VectorXd x_hat_min(n);
	Eigen::MatrixXd P_min(n,n); 
	
	Eigen::MatrixXd Set(n,n+1);		//Set of state and covariance
	
	Eigen::MatrixXd Phi_R(3,3);
	Eigen::MatrixXd G(3,2);
	
	Eigen::MatrixXd tempTrans;
	
	//Propagate state (Robot and Landmark, no change in Landmark)
	x_hat_min.head(3) << v_m*cos(ori),
				v_m*sin(ori),
				w_m;
	
	x_hat_min.head(3) = x_hat_plus.head(3) + dt*x_hat_min.head(3);	
	x_hat_min.tail(n-3) = x_hat_plus.tail(n-3);


	// Set up Phi_R and G
	Phi_R << 1, 0, -dt*v_m*sin(ori),
		0, 1, dt*v_m*cos(ori),
		0, 0, 1;
	
	G << -dt*cos(ori), 0,
		-dt*sin(ori), 0,
		0, -dt;
	

	// Propagate Covariance
	// P_RR
	P_min.block(0,0,3,3) = Phi_R * P_plus.block(0,0,3,3) * Phi_R.transpose() + G * Q * G.transpose();

	// P_RL
	P_min.block(0,3,3,n-3) = Phi_R * P_plus.block(0,3,3,n-3);

	// P_LR
	tempTrans = P_min.block(0,3,3,n-3).transpose();
	P_min.block(3,0,n-3,3) = tempTrans;

	// P_LL are not affected by propagation.
	P_min.block(3,3,n-3,n-3) = P_plus.block(3,3,n-3, n-3);

	// Make sure the matrix is symmetric, may skip this option
	tempTrans = 0.5 * (P_min + P_min.transpose());
	P_min = tempTrans;	

	// Put State vector and Covariance into one matrix and return it.
	// We can parse these out in the main function.
	Set.block(0,0,n,1) = x_hat_min;
	Set.block(0,1,n,n) = P_min;

	return Set;
}