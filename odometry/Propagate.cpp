
#include "kalmanfilter.h"


/*
typedef struct State_Covariance {
	Eigen::VectorXd x_hat_min;
	Eigen::MatrixXd P_min;

}Set;
*/

/* Return a matrix composed of following:
1st column: x_hat_min

From 2nd column to last column: P_min

we can parse those two out in the main function		*/ 

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
		0, 1;
	

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

// //original main function for propagation
// int main2()
// {
// 	int n = 7;			//size of state vector, #of Landmark = (n-3)/2
// 
// 	double v = 0.2;
// 	double w = 0.01;
// 	double sigma_v = 0.01;
// 	double sigma_w = 0.04;
// 
// 	VectorXd x_hat_min;
// 	MatrixXd P_min;
// 	
// 	MatrixXd m(n,n);		//Covariance of state including robot and landmark
// 	MatrixXd Q(2,2);		//Covariance of noise of Control input: linear/rotational velocity
// 	
// 	MatrixXd Set;
// 	
// 	x_hat_min = VectorXd(n);
// 	x_hat_min(2) = pi/2;
// 
// 	//Practice input
// 	m <<  1, 2, 3, 4, 17, 18, 37,
// 		5, 6, 7, 8, 19, 20, 38,
// 		9,10,11,12, 21, 22, 39,
// 		13,14,15,16, 23, 24, 40,
// 		25,26,27,28,29,30, 41,
// 		31,32,33,34,35,36,42,
// 		43,44,45,46,47,48,49;
// 
// 	
// 	m.block(1,1,2,2) << 1, 0, 0, 1;
// 	
// 	// Get the result of Propagation
// 	Q << sigma_v, 0,
// 		0, sigma_w;
// 	Q = (v*v)*Q*Q;
// 	Set = Propagate(x_hat_min, m, 0.1, 0.01, Q, 1);
// 	
// 	// and parse them out to state vector and covariance matrix
// 	x_hat_min = Set.block(0,0,n,1);
// 	P_min = Set.block(0,1,n,n);
// 	
// 	cout << "\nX_hat_min =" << endl;
// 	cout << x_hat_min << endl << endl;
// 	cout << "P_min =" << endl;
// 	cout << P_min << endl;
// 	
// 	//printf("%f\n", sin(pi/2));
// 	
// 
// }