#include "simulator.h"

int Simulator::Time = 0;
int Simulator::RealX = 0;
int Simulator::RealY = 0;
int Simulator::RealPhi = 0;
std::mutex Simulator::mtxPose;


Simulator::Simulator(ArRobot* robot){
  this->robot = robot;
  
  this->robot->lock();
    ArGlobalRetFunctor1<bool, ArRobotPacket *> ph(&(this->updateRealPose));
    this->robot->addPacketHandler(&ph, ArListPos::FIRST);
    //this->robot->comInt(ArCommands::SIM_STAT, 2);
  this->robot->unlock();
}


int Simulator::getRealXYPhi(int* buffer){
  std::cout << "getrealxyphi\n";
  mtxPose.lock();
    buffer[0] = RealX;
    buffer[1] = RealY;
    buffer[2] = RealPhi;
    int curTime = Time;
  mtxPose.unlock();
  
  return curTime;
}


bool Simulator::updateRealPose(ArRobotPacket* pkt)
{
   if(pkt->getID() != 0x62) return false; // SIMSTAT has id 0x62
   
   mtxPose.lock();
    
    int trash;
    trash = pkt->bufToByte();  // unused byte
    trash = pkt->bufToByte();  // unused byte
    trash = pkt->bufToUByte4();
    trash = pkt->bufToUByte2();
    trash = pkt->bufToUByte2();
    
    Time = pkt->bufToUByte2();
    RealX = pkt->bufToByte4();
    RealY = pkt->bufToByte4();
    trash = pkt->bufToByte4();
    RealPhi = pkt->bufToByte4();
   
   mtxPose.unlock();
   return true;
}

