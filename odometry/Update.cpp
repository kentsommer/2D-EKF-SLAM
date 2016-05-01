
#include <iostream>
#include <cmath>
//#include <chrono>
#include <unistd.h>
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <stdio.h>

//#include "kalmanfilter.h"


/*
typedef struct State_Covariance {
	Eigen::VectorXd x_hat_min;
	Eigen::MatrixXd P_min;

}Set;
*/

//Calculating condition number:
//http://stackoverflow.com/questions/33575478/how-can-you-find-the-condition-number-in-eigen

int knownFeatureNumber;

Eigen::MatrixXd addFeature(Eigen::VectorXd x_hat, Eigen::MatrixXd P,double F_X_m,double F_Y_m,Eigen::MatrixXd R) {
	
	double xhat=x_hat(0);
	double yhat=x_hat(1);
	double phihat=x_hat(2);
	Eigen::MatrixXd P_hat=P;
	double fxhat; // Feature global X coordinate
  	double fyhat; // Feature global Y coordinate
  	fxhat=xhat+F_X_m*cos(phihat)-F_Y_m*sin(phihat);
  	fyhat=yhat+F_Y_m*cos(phihat)+F_X_m*sin(phihat);
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
  		std::cout << j << std::endl;

  		Prj=P.block<3,2>(0,3+2*j);
  	
  		Pjr=P.block<3,2>(3+2*j,0);
  		Pjr=Prj.transpose();
  		P.block<2,2>(3+knownFeatureNumber*2,3+2*j)=-Hli.transpose()*Hr*Prj;
  		P.block<2,2>(3+2*j,3+knownFeatureNumber*2)=-Pjr*Hr.transpose()*Hli;
  		
	  }//for
  }//if
  //Prli=-Prr*Hr.transpose()*Hli;
  
  	Eigen::MatrixXd Set (x_hat.size(),x_hat.size()+1);
	Set.block(0,0,x_hat.size(),1) = x_hat;
	Set.block(0,1,x_hat.size(),x_hat.size()) = P;
	
	knownFeatureNumber=knownFeatureNumber+1;
	return Set;
	
	
	
	
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
}


int main(){//testing add feature
	double pi=3.1415926;
	Eigen::MatrixXd P (10,10);
	Eigen::VectorXd x_hat (10);
	x_hat << 1,2,0,0,0,0,0,0,0,0;
	Eigen::MatrixXd Set;
		
	P.block<3,3>(0,0)=Eigen::MatrixXd::Ones(3,3);
	double F_X_m=2;
	double F_Y_m=3;
	knownFeatureNumber=0;
	Eigen::MatrixXd R(2,2);
	R << .1*.1,0,0,.1*.1;
	Set=addFeature(x_hat,P, F_X_m, F_Y_m, R);
	std::cout << ("knownFeatureNumber=") << std::endl;	
	std::cout << knownFeatureNumber << std::endl;
	addFeature(x_hat, P, -1, -1, R);

	
}
