#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <stdio.h>

double inf = 999999999999;
double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;

using namespace std;

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

Eigen::MatrixXd Update(Eigen::VectorXd x_hat_min, Eigen::MatrixXd P_min, Eigen::MatrixXd z_chunk, Eigen::MatrixXd R_chunk, int Gamma_max, int Gamma_min)
{
	
	int i, j, Li;	

	int n_lm = (x_hat_min.size()-3)/2;		// #of landmark in the map
	int n_z = z_chunk.size()/2;				// #of inferred relative position measurements
	
	double phi = x_hat_min(2);	
	double Mahal_dist = inf;
	
	Eigen::MatrixXd I;
	Eigen::MatrixXd J(2,2);
	Eigen::MatrixXd C(2,2);				// Rotational matrix
	
	Eigen::MatrixXd z(2,1);
	Eigen::VectorXd res(2,1);
	Eigen::MatrixXd R(2,2);


	Eigen::VectorXd G_pR_hat(2);
	Eigen::VectorXd G_pLi_hat(2);

	Eigen::VectorXd R_pLi_hat(2);
	Eigen::VectorXd newLand;

	Eigen::MatrixXd H;
	Eigen::MatrixXd H_R;
	Eigen::MatrixXd H_Li;
	Eigen::MatrixXd S;
	Eigen::MatrixXd K;
	

	//Save optimal:
	int Opt_i = 0;
	Eigen::VectorXd Opt_res;
	Eigen::MatrixXd inv_S(2,2);
	Eigen::MatrixXd Opt_H(2, x_hat_min.size());
	Eigen::MatrixXd Opt_H_R;
	Eigen::MatrixXd Opt_H_Li;
	
	Eigen::MatrixXd Set;
	J << 0, -1, 1, 0;
	

	double temp;
	Eigen::MatrixXd temp_inv_S;
	
	
	printf("%d\n", n_z);
	
	for(j=1;j<=n_z;j++)
	{
		//Fix those values by z_j or x_hat_min
		z = z_chunk.block(0,j-1,2,1);
		R = R_chunk.block(0,j*2-2,2,2);
		C << cos(phi), -sin(phi), sin(phi), cos(phi);
		
		H_Li = C.transpose();
		
		G_pR_hat = x_hat_min.head(2);

		//Get the Mahalanobis distance between z_j and all of landmarks,
		//Pick the landmark with minimum Mahalanobis distance and save corresponding H/H_R/H_Li
		Mahal_dist = inf;
		Opt_i = 0;

		for(i=1;i<=n_lm;i++)
		{
			//Estimated position of landmark in the map
			Li = i*2 + 1;		//Li = i*2 + 3 -1 -1;
			
			//std::cout << x_hat_min.segment(Li-1,2) << std::endl;			
			G_pLi_hat = x_hat_min.segment(Li,2);
			
			R_pLi_hat = C.transpose() * (G_pLi_hat - G_pR_hat);
			
			
			H = Eigen::MatrixXd(2,x_hat_min.size());
			H_R = Eigen::MatrixXd(2,3);
			H_R.block(0,0,2,2) = -1*C.transpose();

			H_R.block(0,2,2,1) = -1*C.transpose() * J*(G_pLi_hat - G_pR_hat);
			
			H.block(0,0,2,3) = H_R;
			H.block(0,Li,2,2) = H_Li;

			res = z - R_pLi_hat;
			S = H*P_min*H.transpose() + R;
			S = 0.5*(S + S.transpose());
					
			temp_inv_S = S.inverse();
			temp = res.transpose()*temp_inv_S*res;
			
			if(Mahal_dist > temp)
			{
				
				Mahal_dist = temp;
				Opt_i = i;
				
				Opt_res = res;
				inv_S = temp_inv_S;

				Opt_H = H;
				Opt_H_R = H_R;
				Opt_H_Li= H_Li;
			}
			
		}
		
		
		if(Opt_i == 0 || Mahal_dist > Gamma_max)
		{
			printf("%d: Initialize\n", j);
			newLand = G_pR_hat + C*z;
			//call initialize()
			
		}
		else if(Mahal_dist < Gamma_min)
		{
			printf("%d: Update\n", j);
			res = Opt_res;
			I = Eigen::MatrixXd::Identity(x_hat_min.size(),x_hat_min.size());
			//Inefficient way(Batch)
			
			H = Opt_H;
			K = P_min*H.transpose()*inv_S;
			x_hat_min = x_hat_min + K*res;
			
			P_min = (I - K*H) * P_min * (I - K*H).transpose() + K*R*K.transpose();
			std::cout << x_hat_min << std::endl;
			std::cout << P_min << std::endl;
		}
	}	
	Set = Eigen::MatrixXd(x_hat_min.size(),x_hat_min.size()+1);
	Set.block(0,0,x_hat_min.size(),1) = x_hat_min;
	Set.block(0,1,x_hat_min.size(),x_hat_min.size()) = P_min;
	return Set;
}

int main()
{
	int n = 9;			//size of state vector, #of Landmark = (n-3)/2
	int i;
	double v = 0.2;
	double w = 0.01;
	double sigma_v = 0.01;
	double sigma_w = 0.04;

	Eigen::VectorXd x_hat_min;
	Eigen::MatrixXd P_min;
	
	Eigen::MatrixXd m(n,n);		//Covariance of state including robot and landmark
	Eigen::MatrixXd Q(2,2);		//Covariance of noise of Control input: linear/rotational velocity
	Eigen::MatrixXd Q_chunk(2,n-3);
	Eigen::MatrixXd S;
	Eigen::MatrixXd z_chunk(2,(n-3)/2);
	x_hat_min = Eigen::VectorXd(n);
	x_hat_min(2) = pi/2;

	//Practice input
	m <<  67, 89, 1, 2, 3, 4, 17, 18, 37,
		15, 26, 5, 6, 7, 8, 19, 20, 38,
		66, 22, 9,10,11,12, 21, 22, 39,
		44, 64, 13,14,15,16, 23, 24, 40,
		82, 23, 25,26,27,28,29,30, 41,
		24, 45, 31,32,33,34,35,36,42,
		11, 66, 43,44,45,46,47,48,49,
		15, 26, 5, 6, 7, 8, 19, 20, 38,
		66, 22, 9,10,11,12, 21, 22, 39;

	
	m.block(1,1,2,2) << 1, 0, 0, 1;
	//std::cout << x_hat_min.transpose() << endl;
	//std::cout<<m.size()<<std::endl;
	//std::cout<< x_hat_min.transpose() << endl;

	Q << sigma_v, 0,
      		0, sigma_w;
  	Q = (v*v)*Q*Q;
	for(i=1;i<=(n-3)/2;i++)
		Q_chunk.block(0,i*2-2, 2,2) = Q;

	z_chunk.block(0,0,2,1) << 2,3;
	z_chunk.block(0,1,2,1) << 3,2;
	z_chunk.block(0,2,2,1) << 15,15;
	S = Update(x_hat_min, m, z_chunk, Q_chunk, 1,2);
	
}