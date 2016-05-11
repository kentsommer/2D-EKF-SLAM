#include <iostream>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include <fstream>
#include <time.h>
#include <string>

#include "Aria.h"
#include "odometry/kalmanfilter.h"
#include "movement/movementcontroller.h"
#include "features/featuredetector.h"
#include "features/houghtransform.h"


int main(int argc, char **argv)
{
  // setup outfiles
    // odometry
  std::ofstream odomFile;
  std::string odomfileName = "./data/odom/odomRun.txt";
  std::string odomPath = odomfileName;
  std::remove(odomPath.c_str());
    //scan points
  std::ofstream scanFile;
  std::string scanfileName = "./data/scan/scanRun.txt";
  std::string scanPath = scanfileName;
  std::remove(scanPath.c_str());
    //features (all)
  std::ofstream featuresFile;
  std::string featuresfileName = "./data/features/featuresRun.txt";
  std::string featuresPath = featuresfileName; 
  std::remove(featuresPath.c_str());
     //known features (in state)
  std::ofstream knownfeaturesFile;
  std::string knownfeaturesfileName = "./data/features/knownfeaturesRun.txt";
  std::string knownfeaturesPath = knownfeaturesfileName;
  std::remove(knownfeaturesPath.c_str());
      // covariance
  std::ofstream covFile;
  std::string covfileName = "./data/cov/covRun.txt";
  std::string covPath = covfileName;
  std::remove(covPath.c_str());
    //Open all out files
  odomFile.open(odomPath);
  featuresFile.open(featuresPath);
  knownfeaturesFile.open(knownfeaturesPath);
  scanFile.open(scanPath);
  covFile.open(covPath);


  //Initialize Aria
  Aria::init();
  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();
  ArRobot robot;
  ArSick sick;


  ArSimpleConnector connector(&argc, argv);

  if (!connector.parseArgs() || argc > 1)
  {
    connector.logOptions();
    exit(1);
  }

  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);

  puts("This program performs SLAM");
  
  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);
  robot.addRangeDevice(&sick);

  if (!connector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }

  robot.runAsync(true);

  
  //Initialize and connect to the laser
  sick.configureShort(false,ArSick::BAUD38400,ArSick::DEGREES180,ArSick::INCREMENT_ONE);
  connector.setupLaser(&sick);
  sick.runAsync();


  if (!sick.blockingConnect())
  {
    printf("Could not connect to SICK laser... exiting\n");
    robot.disconnect();
    Aria::shutdown();
    return 1;
  }


  //Enable the moters 
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);


  //Initialize the feature detector
  FeatureDetector* f = new FeatureDetector(&sick);

  //Setup timers to manage dt loop timings
  std::chrono::high_resolution_clock::time_point t1; 
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

  //Initialize the movement controller
  MovementController* mov = new MovementController(&robot, &sick);
  mov->start();

  robot.requestEncoderPackets();
  
  //Set dt
  double dt = 0.001;
  double loopTime = 0.0;

  //Initialize the kalman filter
  KalmanFilter* ekf = new KalmanFilter(&robot);

  //Enter SLAM loop 
  while (1){
    //Propagate the state
    t1 = t2;
    t2 = std::chrono::high_resolution_clock::now();
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    dt = (double)microseconds / 1000000.0;
    ekf->doPropagation(dt, covFile, knownfeaturesFile);
    
    //Get the features
    std::vector<Feature> fvec;
    double compass;
    f->getFeatures(&fvec, &compass, ekf->Phi);
    
    //Update using structual compass
    if (compass != f->NO_COMPASS) {
      std::cout << "Compass: " << compass << std::endl;
      ekf->doUpdateCompass(compass, 0.0005);
    }
    
    //Update using landmarks / features
    for (int i=0; i<fvec.size(); i++){
      //Initialize matrices
      Eigen::MatrixXd z_chunk(2,1);
      Eigen::MatrixXd R(2,2);
      Eigen::MatrixXd R_chunk(2,2);
      Eigen::MatrixXd G(2,2);

      //Get measurment distance and bearing to compute R
      z_chunk << (fvec[i].x/1000.0), (fvec[i].y/1000.0);
      double fx = fvec[i].x/1000.0;
      double fy = fvec[i].y/1000.0;
      double dist = sqrt(fx*fx + fy*fy);
      double bearing = atan2(fy, fx);

      //Compute R
      R << 0.0025, 0, 0, 0.0001;
      G << cos(bearing), -dist * sin(bearing), sin(bearing), dist * cos(bearing);
      R_chunk = G * R * G.transpose();

      std::cout << "Update: ";
      ekf->doUpdate(z_chunk, R_chunk);
      std::cout << ekf->Num_Landmarks << std::endl;
      
      double newX = fx*cos(ekf->Phi) - fy*sin(ekf->Phi);
      double newY = fx*sin(ekf->Phi) + fy*cos(ekf->Phi);
      
      //Save features to file
      featuresFile << newX + ekf->X << " " << newY + ekf->Y << std::endl;
    }
    
    //Save odometry to file
    odomFile << ekf->X << " " << ekf->Y << std::endl;
    
    //If haven't saved laser scan in over a second, save the laser scan
    loopTime += dt;
    if (loopTime > 1.0){      
      sick.lockDevice();
        std::vector<ArSensorReading> *r = sick.getRawReadingsAsVector();
        std::vector<ArSensorReading> readings(*r);
      sick.unlockDevice();
            
      for (int i=0; i<readings.size(); i++){
        if (readings[i].getRange() > 7000) continue;
        
        double fx = readings[i].getLocalX()/1000.0;
        double fy = readings[i].getLocalY()/1000.0;
        double newX = fx*cos(ekf->Phi) - fy*sin(ekf->Phi);
        double newY = fx*sin(ekf->Phi) + fy*cos(ekf->Phi);

        //Save scan to file
        scanFile << newX + ekf->X << " " << newY + ekf->Y << std::endl;
      }
      loopTime = 0.0;
    }
  }
  
  //Wait for robot task loop to end before exiting the program
  robot.waitForRunExit();
  
  //Close all files
  odomFile.close();
  featuresFile.close();
  knownfeaturesFile.close();
  scanFile.close();
  covFile.close();

  Aria::exit(0);
  return 0;
}
