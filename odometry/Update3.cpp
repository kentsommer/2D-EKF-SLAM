#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "kalmanfilter.h"

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




Eigen::MatrixXd KalmanFilter::Update3(Eigen::VectorXd x_hat_min, Eigen::MatrixXd P_min, Eigen::MatrixXd z_chunk, Eigen::MatrixXd R_chunk, int Gamma_max, int Gamma_min)
{
	int stateSize, tempSize;	

	int n_lm = (x_hat_min.size()-3)/2;		// #of landmark in the map
	int n_z = z_chunk.size()/2;				// #of INFerred relative position measurements
	
	double phi;	
	double Mahal_dist = INF;
	
	Eigen::VectorXd tempVector;
	Eigen::MatrixXd tempMatrix;

	Eigen::MatrixXd tempTrans;	
	Eigen::MatrixXd J(2,2);				// J
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


	//blocks of P_min
	Eigen::MatrixXd P_RR;
	Eigen::MatrixXd P_RLi;
	Eigen::MatrixXd P_LiR;
    Eigen::MatrixXd P_LiLi;
	

	//Save optimal:
	int Opt_i = 0;
	Eigen::VectorXd Opt_res;
	Eigen::MatrixXd Opt_H_R;
	Eigen::MatrixXd Opt_H_Li;
	Eigen::MatrixXd Opt_S;
	Eigen::MatrixXd Opt_K;
	
	Eigen::MatrixXd Set;
	J << 0, -1, 1, 0;
	

	double temp;
	Eigen::MatrixXd temp_inv_S;
	
	
	
// 	printf("%d\n", n_z);
	for(int j=1;j<=n_z;j++)
	{

		stateSize = x_hat_min.size();
		//Fix those values by z_j or x_hat_min
		z = z_chunk.block(0,j-1,2,1);
		R = R_chunk.block(0,j*2-2,2,2);
		phi = x_hat_min(2);
		C << cos(phi), -sin(phi), sin(phi), cos(phi);
		
		P_RR = P_min.block(0,0,3,3);
		H_Li = C.transpose();
		
		G_pR_hat = x_hat_min.head(2);

		//Get the Mahalanobis distance between z_j and all of landmarks,
		//Pick the landmark with minimum Mahalanobis distance and save corresponding H/H_R/H_Li
		Mahal_dist = INF;
		Opt_i = 0;

		for(int i=1;i<=n_lm;i++)
		{
			//Estimated position of landmark in the map
			int Li = i*2 + 1;		//Li = i*2 + 3 -1 -1;
			
			//std::cout << x_hat_min.segment(Li-1,2) << std::endl;			
			G_pLi_hat = x_hat_min.segment(Li,2);
			
			R_pLi_hat = C.transpose() * (G_pLi_hat - G_pR_hat);
			
			res = z - R_pLi_hat;
			H_R = Eigen::MatrixXd(2,3);
			H_R.block(0,0,2,2) = -1.0*C.transpose();
			H_R.block(0,2,2,1) = -1.0*C.transpose() * J*(G_pLi_hat - G_pR_hat);
			
			P_RLi = P_min.block(0,Li,3,2);
        		P_LiR = P_min.block(Li,0,2,3);
        		P_LiLi= P_min.block(Li,Li,2,2);
        		
			S = H_R*P_RR*H_R.transpose() + H_Li*P_LiR*H_R.transpose() + H_R*P_RLi*H_Li.transpose() + H_Li*P_LiLi*H_Li.transpose() + R;
			tempTrans = 0.5*(S + S.transpose());
			S = tempTrans;

			//Calculating condition number
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(S);
			double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
			
			if(cond >= 80) continue;
							
			temp_inv_S = S.inverse();
			temp = res.transpose()*temp_inv_S*res;
            
			if(Mahal_dist > temp)
			{
				Mahal_dist = temp;
				Opt_i = Li;
				Opt_res = res;
				Opt_S = S;
				Opt_H_R = H_R;
			}
		}
		
		
		
		if(Mahal_dist < 0.0){
			std::cout << "Dist BAD: " << Mahal_dist << std::endl;
		}
		
		
		
		//new landmark
		if(Opt_i == 0 || Mahal_dist > Gamma_max)
		{
          std::cout << "New ";
			newLand = G_pR_hat + C*z;

			// Update state
			tempVector = x_hat_min;
			x_hat_min = Eigen::VectorXd(stateSize+2);
			x_hat_min.head(stateSize) = tempVector;
			x_hat_min.tail(2) = newLand;
            
            H_R = Eigen::MatrixXd(2,3);
			// Update Covariance
			H_R.block(0,0,2,2) = -1.0*C.transpose();
			H_R.block(0,2,2,1) = -1.0*C.transpose() * J*(newLand - G_pR_hat);

			P_LiLi = H_Li.transpose() * (H_R *P_min.block(0,0,3,3) * H_R.transpose() + R) * H_Li;
			P_RLi = -P_min.block(0,0, stateSize, 3) * H_R.transpose() * H_Li;
			tempMatrix = P_min;
			
			P_min = Eigen::MatrixXd(stateSize+2,stateSize+2);
			
			P_min.block(0,0,stateSize, stateSize) = tempMatrix;
			P_min.block(0,stateSize, stateSize, 2) = P_RLi;
			P_min.block(stateSize,0, 2, stateSize) = P_RLi.transpose();
			P_min.block(stateSize, stateSize, 2,2) = P_LiLi;
		}
		
		
		
		//existing landmark
		else if(Mahal_dist < Gamma_min)
		{
          std::cout << "Old ";
			//Calculating condition number
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(Opt_S.inverse());
			double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
			
			if(cond >= 80)
			{
				printf("%d: Bad measurement covariance\n", j);
				continue;
			}
			
			//efficient way(block Operation)
			K = (P_min.block(0,0,stateSize,3)*Opt_H_R.transpose() + P_min.block(0,Opt_i,stateSize,2)*H_Li.transpose()) * Opt_S.inverse();
			x_hat_min = x_hat_min + K*Opt_res;
			P_min = P_min - K*Opt_S*K.transpose();
		}
		
		else std::cout << "Ign ";
		
		tempTrans = 0.5*(P_min + P_min.transpose());
		P_min = tempTrans;
	}
	
	
	
	Set = Eigen::MatrixXd(x_hat_min.size(),x_hat_min.size()+1);
	Set.block(0,0,x_hat_min.size(),1) = x_hat_min;
	Set.block(0,1,x_hat_min.size(),x_hat_min.size()) = P_min;
    
	return Set;
}
