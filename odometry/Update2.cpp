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

Eigen::MatrixXd KalmanFilter::addFeature2(Eigen::VectorXd x_hat, Eigen::MatrixXd P, Eigen::VectorXd newLand,Eigen::MatrixXd R) {
	
	int knownFeatureNumber = (x_hat.size()-3)/2;
	double F_X_m = newLand(0);
	double F_Y_m = newLand(1);
	double xhat=x_hat(0);
	double yhat=x_hat(1);
	double phihat=x_hat(2);
	
    Eigen::MatrixXd P_hat=P;
    P = Eigen::MatrixXd(5 + knownFeatureNumber*2, 5 + knownFeatureNumber*2);
    P.block(0,0,3+knownFeatureNumber*2,3+knownFeatureNumber*2) = P_hat;
    
	double fxhat; // Feature global X coordinate
  	double fyhat; // Feature global Y coordinate
  	fxhat=xhat+F_X_m*cos(phihat)-F_Y_m*sin(phihat);
  	fyhat=yhat+F_Y_m*cos(phihat)+F_X_m*sin(phihat);
    Eigen::VectorXd x_hat_temp = x_hat;
    x_hat = Eigen::VectorXd(5+knownFeatureNumber*2);
    x_hat.head(3+knownFeatureNumber*2) = x_hat_temp;
  	x_hat(3+knownFeatureNumber*2)=fxhat;
  	x_hat(4+knownFeatureNumber*2)=fyhat;
  	
  	Eigen::MatrixXd Hr(2,3);
  	Eigen::MatrixXd Hli(2,2);
  	Eigen::MatrixXd Prr(3,3);
	  Prr=P.block<3,3>(0,0);
	  
  	Hr<<-(2*fxhat - 2*xhat)/(2*sqrt(((fxhat - xhat)*(fxhat - xhat) + (fyhat - yhat)*(fyhat - yhat)))),-(2*fyhat - 2*yhat)/(2*sqrt(((fxhat - xhat)*(fxhat - xhat) + (fyhat - yhat)*(fyhat - yhat)))),0,
     (fyhat - yhat)/((fxhat - xhat)*(fxhat - xhat) + (fyhat - yhat)*(fyhat - yhat)),-(fxhat - xhat)/((fxhat - xhat)*(fxhat - xhat) + (fyhat - yhat)*(fyhat - yhat)),-1;
  
   // fxhat=(fxhat+fyhat)*(fxhat+fyhat);
    Hli<<(2*fxhat - 2*xhat)/(2*sqrt(((fxhat - xhat)*(fxhat - xhat) + (fyhat - yhat)*(fyhat - yhat)))),(2*fyhat - 2*yhat)/(2*sqrt(((fxhat - xhat)*(fxhat - xhat) + (fyhat - yhat)*(fyhat - yhat)))),
    -(fyhat - yhat)/((fxhat - xhat)*(fxhat - xhat) + (fyhat - yhat)*(fyhat - yhat)),(fxhat - xhat)/((fxhat - xhat)*(fxhat - xhat) + (fyhat - yhat)*(fyhat - yhat));
        
 // Prr.block<2,2>(0,0)<<1, 1, 1, 1;
 // P(1:3,4+knownFeaturesNumber*2:5+knownFeaturesNumber*2)=-Prr*Hr'*Hli;
  P.block<3,2>(0,3+knownFeatureNumber*2)=-Prr*Hr.transpose()*Hli;
  P.block<2,3>(3+knownFeatureNumber*2,0)=-Hli.transpose()*Hr*Prr;
  P.block<2,2>(3+knownFeatureNumber*2,3+knownFeatureNumber*2)=Hli.transpose()*(Hr*Prr*Hr.transpose()+R)*Hli;
	Eigen::MatrixXd Prj(3,2);
    Eigen::MatrixXd Pjr(2,3);
    
  if (knownFeatureNumber>0){
  	for (int j=0;j<knownFeatureNumber;j=j+1){
//   		std::cout << j << std::endl;

  		Prj=P.block<3,2>(0,3+2*j);
  	
  		Pjr=P.block<2,3>(3+2*j,0); //3,2
  		//Prj.transposeInPlace();
        
  		P.block(3+knownFeatureNumber*2,3+2*j,2,2)=-Hli.transpose()*Hr*Prj;
  		P.block(3+2*j,3+knownFeatureNumber*2,2,2)=-Pjr*Hr.transpose()*Hli;
  		
	  }//for
  }//if
  //Prli=-Prr*Hr.transpose()*Hli;
  
  	Eigen::MatrixXd Set (x_hat.size(),x_hat.size()+1);
	Set.block(0,0,x_hat.size(),1) = x_hat;
	Set.block(0,1,x_hat.size(),x_hat.size()) = P;
	
	knownFeatureNumber=knownFeatureNumber+1;
	return Set;
	
	
	
/*	
  std::cout << "knownFeatureNumber=" << std::endl;
  std::cout << knownFeatureNumber << std::endl;
  std::cout << "Hli=" << std::endl;
  std::cout << (Hli) << std::endl;
  std::cout << "R =" << std::endl;
  std::cout << (R) << std::endl;
  std::cout << "P =" << std::endl;
  std::cout << P << std::endl;
  
  
  std::cout << "Prr=" << std::endl;
  std::cout << (Prr) << std::endl;
  std::cout << "Fy=" << std::endl;
  std::cout << (fyhat) << std::endl;
*/
}



Eigen::MatrixXd KalmanFilter::Update2(Eigen::VectorXd x_hat_min, Eigen::MatrixXd P_min, Eigen::MatrixXd z_chunk, Eigen::MatrixXd R_chunk, int Gamma_max, int Gamma_min)
{
	int i, j, Li, stateSize, tempSize;	

	int n_lm = (x_hat_min.size()-3)/2;		// #of landmark in the map
	int n_z = z_chunk.size()/2;				// #of INFerred relative position measurements
	
	double phi = x_hat_min(2);	
	double Mahal_dist = INF;

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
	for(j=1;j<=n_z;j++)
	{

		stateSize = x_hat_min.size();
		//Fix those values by z_j or x_hat_min
		z = z_chunk.block(0,j-1,2,1);
		R = R_chunk.block(0,j*2-2,2,2);
		C << cos(phi), -sin(phi), sin(phi), cos(phi);
		
		P_RR = P_min.block(0,0,3,3);
		H_Li = C.transpose();
		
		G_pR_hat = x_hat_min.head(2);

		//Get the Mahalanobis distance between z_j and all of landmarks,
		//Pick the landmark with minimum Mahalanobis distance and save corresponding H/H_R/H_Li
		Mahal_dist = INF;
		Opt_i = 0;

		for(i=1;i<=n_lm;i++)
		{
			//Estimated position of landmark in the map
			Li = i*2 + 1;		//Li = i*2 + 3 -1 -1;
			
			//std::cout << x_hat_min.segment(Li-1,2) << std::endl;			
			G_pLi_hat = x_hat_min.segment(Li,2);
			
			R_pLi_hat = C.transpose() * (G_pLi_hat - G_pR_hat);
			
			res = z - R_pLi_hat;
			H_R = Eigen::MatrixXd(2,3);
			H_R.block(0,0,2,2) = -1*C.transpose();
			H_R.block(0,2,2,1) = -1*C.transpose() * J*(G_pLi_hat - G_pR_hat);
			
			P_RLi = P_min.block(0,Li,3,2);
        		P_LiR = P_min.block(Li,0,2,3);
        		P_LiLi= P_min.block(Li,Li,2,2);
        		
			S = H_R*P_RR*H_R.transpose() + H_Li*P_LiR*H_R.transpose() + H_R*P_RLi*H_Li.transpose() + H_Li*P_LiLi*H_Li.transpose() + R;
			tempTrans = 0.5*(S + S.transpose());
			S = tempTrans;	
							
			temp_inv_S = S.inverse();
			temp = res.transpose()*temp_inv_S*res;
			if(Mahal_dist > temp)
			{
				//printf("%d landmark selected\n", i);
				//std::cout << H_R << std::endl;
				//std::cout << S << std::endl;
				Mahal_dist = temp;
				Opt_i = i;
// 				std::cout<<S<<std::endl;
				Opt_res = res;
				Opt_S = S;
				Opt_H_R = H_R;
				Opt_H_Li= H_Li;
				Opt_K = (P_min.block(0,0,stateSize,3)*H_R.transpose() + P_min.block(0,Li,stateSize,2)*H_Li.transpose())*temp_inv_S;
			}
			
		}
		
		std::cout << "Dist: " << Mahal_dist << std::endl;
		
		if(Opt_i == 0 || Mahal_dist > Gamma_max)
		{
			printf("%d: Initialize\n", j);
			newLand = G_pR_hat + C*z;
			//call initialize()
			Set = KalmanFilter::addFeature2(x_hat_min, P_min, newLand, R);
			
			//get Size
			tempSize = sqrt(Set.size());
			x_hat_min = Set.block(0,0,tempSize,1);
			P_min = Set.block(0,1,tempSize,tempSize);
			
		}
		else if(Mahal_dist < Gamma_min)
		{
			//Calculating condition number
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(Opt_S.inverse());
			double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
			
			if(cond >= 80)
			{
				printf("%d: Bad measurement covariance\n", j);
				continue;
			}
	
			printf("%d: Update\n", j);
			
			//efficient way(block Operation)
			res = Opt_res;
			S = Opt_S;
			K = Opt_K;
			x_hat_min = x_hat_min + K*res;
			P_min = P_min - K*S*K.transpose();
			tempTrans = 0.5*(P_min + P_min.transpose());
			P_min = tempTrans;
			
			//std::cout << "Kalman Gain:" << std::endl;
 			//std::cout << K <<std::endl;
// 			std::cout << "state: " << std::endl;
// 			std::cout << x_hat_min << std::endl;
// 			std::cout << "Covarinace: "<< std::endl;
// 			std::cout << P_min << std::endl;
		}
	}	
	
	Set = Eigen::MatrixXd(x_hat_min.size(),x_hat_min.size()+1);
	Set.block(0,0,x_hat_min.size(),1) = x_hat_min;
	Set.block(0,1,x_hat_min.size(),x_hat_min.size()) = P_min;
    
    std::cout << "Num Landmarks: " << (x_hat_min.size() - 3)/2 << std::endl;
	return Set;
}

/*int main()
{
	int n = 9;			//size of state vector, #of Landmark = (n-3)/2
	int i;
	double v = 0.2;
	double w = 0.01;
	double sigma_v = 0.01;
	double sigma_w = 0.04;

	Eigen::VectorXd x_hat_min;
	Eigen::MatrixXd P_min;
	Eigen::MatrixXd tempTrans;	
	Eigen::MatrixXd m(n,n);		//Covariance of state including robot and landmark
	Eigen::MatrixXd Q(2,2);		//Covariance of noise of Control input: linear/rotational velocity
	Eigen::MatrixXd Q_chunk(2,n-3);
	Eigen::MatrixXd S;
	Eigen::MatrixXd z_chunk(2,(n-3)/2);
	x_hat_min = Eigen::VectorXd(n);
	x_hat_min(2) = PI/2;

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
	tempTrans = 0.5*(m + m.transpose());
	m = tempTrans;

	Q << sigma_v, 0,
      		0, sigma_w;
  	Q = (v*v)*Q*Q;
	for(i=1;i<=(n-3)/2;i++)
		Q_chunk.block(0,i*2-2, 2,2) = Q;

	z_chunk.block(0,0,2,1) << 2,3;
	z_chunk.block(0,1,2,1) << 3,2;
	z_chunk.block(0,2,2,1) << 15,15;
	S = Update(x_hat_min, m, z_chunk, Q_chunk, 1,2);
	
}*/
