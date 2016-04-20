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

#include "Aria.h"
#include "odometry/kalmanfilter.h"
#include "simulator/simulator.h"


int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();
  ArRobot robot;
  ArRobotConnector robotConnector(&argParser, &robot);
  ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "Could not connect to the robot.");
    if(argParser.checkHelpAndWarnUnparsed())
    {
        // -help not given, just exit.
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }


  // Trigger argument parsing
  if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }

  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);

  puts("This program will make the robot wander around. It uses some avoidance\n"
  "actions if obstacles are detected, otherwise it just has a\n"
  "constant forward velocity.\n\nPress CTRL-C or Escape to exit.");
  
  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);

  robot.runAsync(true);

  
  // try to connect to laser. if fail, warn but continue, using sonar only
  if(!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
  }


  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);
  
  robot.lock();
  robot.setVel2(200, 150);
  robot.unlock();
  //*/

  // add a set of actions that combine together to effect the wander behavior
  /*ArActionStallRecover recover;
  ArActionBumpers bumpers;
  ArActionAvoidFront avoidFrontNear("Avoid Front Near", 225, 0);
  ArActionAvoidFront avoidFrontFar;
  ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
  robot.addAction(&recover, 100);
  robot.addAction(&bumpers, 75);
  robot.addAction(&avoidFrontNear, 50);
  robot.addAction(&avoidFrontFar, 49);
  robot.addAction(&constantVelocity, 25);//*/
  
  robot.requestEncoderPackets();
  
//   double x1 = 0;
//   double y1 = 0;
//   double phi1 = 0;
  double dt = 0.001;
/*  
  double rx, ry, rphi;
  double x2, y2, phi2;*/
  
  std::chrono::high_resolution_clock::time_point t1; 
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  
  KalmanFilter* ekf = new KalmanFilter(&robot);
  int* buffer = new int[3];
  Simulator* sim = new Simulator(&robot);
  std::cout << "Time, RealX, RealY, RealPhi, EKFx, EKFy, EKFphi\n";
  
  
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
    
    t1 = t2;
    t2 = std::chrono::high_resolution_clock::now();
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    dt = (double)microseconds / 1000000.0;
    //ekf->propagate((int)dt);
    
    
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
    
    int time = sim->getRealXYPhi(buffer);
    
    std::cout << time << ", ";
    std::cout << buffer[0] << ", ";
    std::cout << buffer[1] << ", ";
    std::cout << buffer[2] << ", ";
    std::cout << ekf->X << ", ";
    std::cout << ekf->Y << ", ";
    std::cout << ekf->Phi*180.0/3.141592654 << ", ";
    std::cout << std::endl;
    //std::cout << RTV << std::endl;
    //std::cout << L << ", " << R << ", " << V << ", " << LV << ", " << RV << std::endl;
    usleep(1000);
  }
  
  // wait for robot task loop to end before exiting the program
  robot.waitForRunExit();
  
  Aria::exit(0);
  return 0;
}