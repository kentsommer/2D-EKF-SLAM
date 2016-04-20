#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <mutex>
#include <iostream>
#include "Aria.h"

class Simulator
{
public:
  Simulator(ArRobot* robot);
  int getRealXYPhi(int* buffer);
  
  ArRobot* robot;
  static int Time;
  static int RealX, RealY, RealPhi;
  static std::mutex mtxPose;

private:
  static bool updateRealPose(ArRobotPacket* pkt);
};
#endif // SIMULATOR_H
