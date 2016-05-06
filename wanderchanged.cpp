/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2014 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
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


////////////////////////////////////////
//Packet handler for real simulation x, y, phi, time
int realTime, realint, simint;
int realX, realY, realZ, realPhi;

bool updateRealPose(ArRobotPacket* pkt)
{
  if(pkt->getID() != 0x62) return false; // SIMSTAT has id 0x62
  
   char a = pkt->bufToByte();  // unused byte
   char b = pkt->bufToByte();  // unused byte
   
   ArTypes::UByte4 flags = pkt->bufToUByte4();
   simint = pkt->bufToUByte2();
   realint = pkt->bufToUByte2();
   realTime = pkt->bufToUByte2();
  
   realX = pkt->bufToByte4();
   realY = pkt->bufToByte4();
   realZ = pkt->bufToByte4();
   realPhi = pkt->bufToByte4();
   
   return true;
}
//////////////////////////////////////


//////////////////////////////////////
// Date for filename appending
std::string get_date(void)
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer [80];

  time (&rawtime);
  timeinfo = localtime (&rawtime);

  strftime (buffer,80,"_%F_%I:%M%p.txt",timeinfo);

  return std::string(buffer);
}


int main(int argc, char **argv)
{
  // setup outfiles
    // date
  std::string dateTime = get_date();
    // odometry
  std::ofstream odomFile;
  std::string odomfileName = "./data/odom/odomRun.txt";
  std::string odomPath = odomfileName; // + dateTime;
  std::remove(odomPath.c_str());
    //scan points
  std::ofstream scanFile;
  std::string scanfileName = "./data/scan/scanRun.txt";
  std::string scanPath = scanfileName; // + dateTime;
  std::remove(scanPath.c_str());
    //features
  std::ofstream featuresFile;
  std::string featuresfileName = "./data/features/featuresRun.txt";
  std::string featuresPath = featuresfileName; // + dateTime;
  std::remove(featuresPath.c_str());
   //features in state vector
  std::ofstream knownfeaturesFile;
  std::string knownfeaturesfileName = "./data/features/knownfeatures.txt";
  std::string knownfeaturesPath = knownfeaturesfileName; // + dateTime;
  std::remove(knownfeaturesPath.c_str());
  	// covariance
  std::ofstream covFile;
  std::string covfileName = "./data/cov/covRun.txt";
  std::string covPath = covfileName;
  std::remove(covPath.c_str());
    //Open all out files
  odomFile.open(odomPath);
  featuresFile.open(featuresPath);
  scanFile.open(scanPath);
  covFile.open(covPath);

  Aria::init();
  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();
  ArRobot robot;
  ArSick sick;
  //ArRobotConnector robotConnector(&argParser, &robot);
  //ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

  ArSimpleConnector connector(&argc, argv);

  if (!connector.parseArgs() || argc > 1)
  {
    connector.logOptions();
    exit(1);
  }

/*  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "Could not connect to the robot.");
    if(argParser.checkHelpAndWarnUnparsed())
    {
        // -help not given, just exit.
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }*/


  // Trigger argument parsing
/*  if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }*/

  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);

  puts("This program will make the robot wander around. It uses some avoidance\n"
  "actions if obstacles are detected, otherwise it just has a\n"
  "constant forward velocity.\n\nPress CTRL-C or Escape to exit.");
  
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

  
  // now set up the laser
  sick.configureShort(false,ArSick::BAUD38400,ArSick::DEGREES180,ArSick::INCREMENT_ONE);
  connector.setupLaser(&sick);


  // now that we're connected to the robot, connect to the laser
  sick.runAsync();


  if (!sick.blockingConnect())
  {
    printf("Could not connect to SICK laser... exiting\n");
    robot.disconnect();
    Aria::shutdown();
    return 1;
  }


  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);

/*  robot.lock();
  robot.setVel2(250,150);
  robot.unlock();*/


  // setup new FeatureDetector
  FeatureDetector* f = new FeatureDetector(&sick);
  //f->start();

  // setup timers
  std::chrono::high_resolution_clock::time_point t1; 
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

  // setup movement controller and start it
  MovementController* mov = new MovementController(&robot, &sick);
  mov->start();
  std::cout << "Started\n";


  robot.requestEncoderPackets();
  
  // set dt
  double dt = 0.001;
  double loopTime = 0.0;

  // setup new Kalman Filter
  KalmanFilter* ekf = new KalmanFilter(&robot);


  
  while (1){
    // Propagation
    t1 = t2;
    t2 = std::chrono::high_resolution_clock::now();
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    dt = (double)microseconds / 1000000.0;
    ekf->doPropagation(dt, covFile,knownfeaturesFile);
    
    //measurements
    std::vector<Feature> fvec;
    f->getFeatures(&fvec, nullptr);
    
    for (int i=0; i<fvec.size(); i++){
      //std::cout << "Updt\n";
      //updates
      Eigen::MatrixXd z_chunk(2,1);
      z_chunk << (fvec[i].x/1000.0), (fvec[i].y/1000.0);
      Eigen::MatrixXd R(2,2);
      Eigen::MatrixXd R_chunk(2,2);
      Eigen::MatrixXd G(2,2);
      //R_chunk << 0.000625, 0, 0, 0.000625;
      double fx = fvec[i].x/1000.0;
      double fy = fvec[i].y/1000.0;
      double dist = sqrt(fx*fx + fy*fy);
      double bearing = atan2(fy, fx);

/*      std::cout << "The bear haha haha...: " << bearing << std::endl;
      std::cout << "The dist is          : " << dist << std::endl;*/

      R << 0.0001, 0, 0, 0.0001;
      G << cos(bearing), -dist * sin(bearing), sin(bearing), dist * cos(bearing);
      //R_chunk << 0.0220206449216767, 0, 0, 0.0220206449216767;
      R_chunk = G * R * G.transpose();
      ekf->doUpdate(z_chunk, R_chunk);
      
      double newX = fx*cos(ekf->Phi) - fy*sin(ekf->Phi);
      double newY = fx*sin(ekf->Phi) + fy*cos(ekf->Phi);
      
      featuresFile << newX + ekf->X << " " << newY + ekf->Y << std::endl;
    }
    


//     std::cout << ekf->X << " " << ekf->Y << " " << ekf->Phi << std::endl;
    odomFile << ekf->X << " " << ekf->Y << std::endl;
    
    loopTime += dt;
    if (loopTime > 1.0){      
      sick.lockDevice();
        std::vector<ArSensorReading> *r = sick.getRawReadingsAsVector();
        std::vector<ArSensorReading> readings(*r);
      sick.unlockDevice();
            
      for (int i=0; i<readings.size(); i++){
        if (readings[i].getRange() > 10000) continue;
        
        double fx = readings[i].getLocalX()/1000.0;
        double fy = readings[i].getLocalY()/1000.0;
        double newX = fx*cos(ekf->Phi) - fy*sin(ekf->Phi);
        double newY = fx*sin(ekf->Phi) + fy*cos(ekf->Phi);
        scanFile << newX + ekf->X << " " << newY + ekf->Y << std::endl;
      }
      
      loopTime = 0.0;
    }
    
    
    //std::cout << "We have a feature at" << std::endl;
      //std::cout << "Time is: " << get_date() << std::endl;
    
//     for (int i=0; i<fvec.size(); i++){
//       std::cout << fvec[i].x << " ";
//       std::cout << fvec[i].y << " ";
//     }
//     std::cout << std::endl;
  }
  // close all out files
  odomFile.close();
  featuresFile.close();
  scanFile.close();
  covFile.close();
  return 0;
  
  
/*  MovementController* mov = new MovementController(&robot, &sick);
  mov->start();
  std::cout << "Started\n";
  mov->join();
  std::cout << "Joined\n";
  robot.waitForRunExit();
  std::cout << "waiting\n";
  Aria::exit(0);
  std::cout << "Aria done\n";
  return 0;*/
  
  
  /*
  // add a set of actions that combine together to effect the wander behavior
  ArActionStallRecover recover;
  ArActionBumpers bumpers;
  ArActionAvoidFront avoidFrontNear("Avoid Front Near", 225, 0);
  ArActionAvoidFront avoidFrontFar;
  ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
  robot.addAction(&recover, 100);
  robot.addAction(&bumpers, 75);
  robot.addAction(&avoidFrontNear, 50);
  robot.addAction(&avoidFrontFar, 49);
  robot.addAction(&constantVelocity, 25);//*/
  
  //robot.requestEncoderPackets();
  
//   double x1 = 0;
//   double y1 = 0;
//   double phi1 = 0;
  //double dt = 0.001;
/*  
  double rx, ry, rphi;
  double x2, y2, phi2;*/
  
  //std::chrono::high_resolution_clock::time_point t1; 
  //std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  
  //KalmanFilter* ekf = new KalmanFilter(&robot);
  
  //register packet handler
  ArGlobalRetFunctor1<bool, ArRobotPacket *> ph(&updateRealPose);
  robot.addPacketHandler(&ph);
  robot.comInt(ArCommands::SIM_STAT, 2);
  
  std::cout << "Time, RealX, RealY, RealPhi, EKFx, EKFy, EKFphi\n";
  
  
  robot.lock();
  robot.setVel2(200, 150);
  robot.unlock();
  while (1) {
//     robot.lock();
//     long L = robot.getLeftEncoder();
//     long R = robot.getRightEncoder();
//     double V = robot.getVel();
//     double RV = robot.getRightVel();
//     double LV = robot.getLeftVel();
//     double RTV = robot.getRotVel() * 3.141592654 / 180.0;
//     rx = robot.getX();
//     ry = robot.getY();
//     //*/
//     robot.unlock();
    
/*    t1 = t2;
    t2 = std::chrono::high_resolution_clock::now();
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    dt = (double)microseconds / 1000000.0;
    ekf->doPropagation(dt);*/
    
    
//     x2 = x1 + dt*V*cos(phi1);
//     y2 = y1 + dt*V*sin(phi1);
//     phi2 = phi1 + dt*RTV;
//     //*/
//     
//     if (phi2 > 2*3.141592654) phi2 -= (3.141592654*2);
//     if (phi2 < 0.0) phi2 += (3.141592654*2);
//     
//     x1 = x2;
//     y1 = y2;
//     phi1 = phi2;
    
//     ArRobotPacket pkt;
//     pkt.setID(ArCommands::SIM_STAT);
//     pkt.uByteToBuf(0); // argument type: ignored.
//     pkt.byte4ToBuf(0);
//     pkt.byte4ToBuf(0);
//     pkt.byte4ToBuf(0);
//     pkt.finalizePacket();
//     robot.getDeviceConnection()->write(pkt.getBuf(), pkt.getLength());
        
//     std::cout << dt << ", ";
//     std::cout << realX << ", ";
//     std::cout << realY << ", ";
//     std::cout << realPhi << ", ";
//     std::cout << ekf->X << ", ";
//     std::cout << ekf->Y << ", ";
//     std::cout << ekf->Phi*180.0/3.141592654 << ", ";
//     std::cout << std::endl;
    //std::cout << RTV << std::endl;
    //std::cout << L << ", " << R << ", " << V << ", " << LV << ", " << RV << std::endl;
    usleep(1000);
  }
  
  // wait for robot task loop to end before exiting the program
  robot.waitForRunExit();
  
  Aria::exit(0);
  return 0;
}
