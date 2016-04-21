#include "movementcontroller.h"
#include "Aria.h"

#define PI 3.14159265
#define FRONT_WALL_THRESHOLD 3000

MovementController::MovementController(ArRobot* robot, ArSick* sick){
  this->robot = robot;
  this->laserScanner = sick;
}

MovementController::~MovementController(){
}

void MovementController::start(){
  struct robot_info* info = new struct robot_info;
  info->robot = robot;
  info->sick = laserScanner;
  int tc;
  
  tc = pthread_create(&thread, nullptr, move_control, (void*)info);
  if (tc) std::cout << "Thread creation Error\n";
}

void MovementController::join(){
  pthread_join(thread, nullptr);
}


void* move_control(void* args){
  struct robot_info* info;
  info = (struct robot_info*)args;


  while(1){
/*    info->sick->lockDevice();
    double d2 = info->sick->getMaxRange();
    info->sick->unlockDevice();
    double dist = getClosestReading(info->sick);*/
    if(shouldTurn(info->sick)){
      std::cout << "SHOULD TURN, WE ARE LESS THAN 3 METERS" << "\n";
    }
    else{
      std::cout << "KEEP DRIVINIG" << "\n";
    }
    //std::cout << "Max Range is: " << d2/1000.0 << "\n";
    usleep(100000);
  }

  move_forward(info->robot);
  usleep(3000000);
  turn_left(info->robot);
  usleep(3000000);
  move_forward(info->robot);
  usleep(3000000);
  turn_left(info->robot);
  usleep(3000000);
  stop(info->robot);

  delete info;
  pthread_exit(nullptr);
}



void move_forward(ArRobot* robot){
  robot->lock();
    robot->setVel2(FORWARD_VEL, FORWARD_VEL);
  robot->unlock();
}

void move_backward(ArRobot* robot){
  robot->lock();
    robot->setVel2(-FORWARD_VEL, -FORWARD_VEL);
  robot->unlock();
}

void turn_left(ArRobot* robot){
  robot->lock();
    robot->setVel2(-TURN_VEL, TURN_VEL);
  robot->unlock();
}

void turn_right(ArRobot* robot){  
  robot->lock();
    robot->setVel2(TURN_VEL, -TURN_VEL);
  robot->unlock();
}

void stop(ArRobot* robot){  
  robot->lock();
    robot->setVel2(0, 0);
  robot->unlock();
}

double getClosestReading(ArSick* sick){
  sick->lockDevice();
    // There is a utility to find the closest reading wthin a range of degrees around the sick, here we use this sick's full field of view (start to end)
    // If there are no valid closest readings within the given range, dist will be greater than sick->getMaxRange().
    double angle = 0;
    double dist = sick->currentReadingPolar(sick->getStartDegrees(), sick->getEndDegrees(), &angle);

    std::cout << "Closest reading is at " << angle << " degrees and is " << dist/1000.0 << " meters away.\n";

  sick->unlockDevice();
  return dist;
}

bool shouldTurn(ArSick* sick)
{
    float distToFrontWall = 0;
    bool shouldTurn = false;
    float pi = 3.14159265;
    sick->lockDevice();
    std::vector<ArSensorReading> *readings = sick->getRawReadingsAsVector();
    sick->unlockDevice();
    
    for(int i=0;i<=20;i=i+2)
    {
        if(readings->size() != 0)
        {
          distToFrontWall += fabs(((*readings)[80+i].getRange())*sin((80+i)*pi/180));
        }
    }
    distToFrontWall/=10; // Average the distnace
    if (distToFrontWall < 3000) // 3 Meters, need to figure out what distance is good
    {
        shouldTurn = true;
    }
    return shouldTurn; 
}

