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
    if(shouldTurn(info->sick)){
      stop(info->robot);
    }
    else{
      move_forward(info->robot);
    }
    alignToWall(info->robot, info->sick);

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

//Currently this just averages, should probably rewrite to check averages on both sides
//Individually so as to avoid running half the robot into a wall :D
bool shouldTurn(ArSick* sick){
  float distToFrontWall = 0;
  bool shouldTurn = false;
  float pi = 3.14159265;
  sick->lockDevice();
  std::vector<ArSensorReading> *readings = sick->getRawReadingsAsVector();
  sick->unlockDevice();
  
  //Sample every 2nd reading from 80 + 
  for(int i=0;i<=20;i=i+2){
      if(readings->size() != 0){
        distToFrontWall += fabs(((*readings)[80+i].getRange())*sin((80+i)*pi/180));
      }
  }
  distToFrontWall/=10; // Average the distnace
  if (distToFrontWall < 1000){ // 1 Meters, need to figure out what distance is good
      shouldTurn = true;
  }
  return shouldTurn; 
}


bool canAlignRight(ArSick* sick){
  sick->lockDevice();
  std::vector<ArSensorReading> *readings = sick->getRawReadingsAsVector();
  sick->unlockDevice();
  if(readings->size() != 0){
    // if the standard deviation of the slopes is very small then they are same line (wall)
    // Can use to align
    float slope1 = ((*readings)[160].getLocalY() - (*readings)[170].getLocalY())/((*readings)[160].getLocalX() - (*readings)[170].getLocalX());
    float slope2 = ((*readings)[150].getLocalY() - (*readings)[170].getLocalY())/((*readings)[150].getLocalX() - (*readings)[170].getLocalX());
    float slope3 = ((*readings)[150].getLocalY() - (*readings)[160].getLocalY())/((*readings)[150].getLocalX() - (*readings)[160].getLocalX());
    float Ex2 = (slope1*slope1+slope2*slope2+slope3*slope3)/3;
    float Ex = (slope1+slope2+slope3)/3;
    float stdDeviation = sqrt(Ex2 - Ex*Ex);
    if (stdDeviation < 0.015){
      return true;
    }
  }
  return false;
}


bool canAlignLeft(ArSick* sick){
  sick->lockDevice();
  std::vector<ArSensorReading> *readings = sick->getRawReadingsAsVector();
  sick->unlockDevice();
  if(readings->size() != 0){
    // if the standard deviation of the slopes is very small then they are same line (wall)
    // Can use to align
    float slope1 = ((*readings)[20].getLocalY() - (*readings)[10].getLocalY())/((*readings)[20].getLocalX() - (*readings)[10].getLocalX());
    float slope2 = ((*readings)[30].getLocalY() - (*readings)[10].getLocalY())/((*readings)[30].getLocalX() - (*readings)[10].getLocalX());
    float slope3 = ((*readings)[30].getLocalY() - (*readings)[20].getLocalY())/((*readings)[30].getLocalX() - (*readings)[20].getLocalX());
    float Ex2 = (slope1*slope1+slope2*slope2+slope3*slope3)/3;
    float Ex = (slope1+slope2+slope3)/3;
    float stdDeviation = sqrt(Ex2 - Ex*Ex);
    if (stdDeviation < 0.015){
      return true;
    }
  }
  return false;
}


void alignToWall(ArRobot* robot, ArSick* sick){
  sick->lockDevice();
  std::vector<ArSensorReading> *readings = sick->getRawReadingsAsVector();
  sick->unlockDevice();

  float distToLeftThresh = 970; //Chosen 
  float distToRightThresh = 970;
  float thThresh = 2;
  float correctionAngle = 0;
  
  if (canAlignRight(sick) && canAlignLeft(sick) && readings->size() != 0){
    std::cout << "Aligning to Both Lines" << "\n";
    float measuredRight = ((*readings)[155].getLocalY() - (*readings)[170].getLocalY())/((*readings)[155].getLocalX() - (*readings)[170].getLocalX());
    float thRight = atan(measuredRight)*180/PI;
    float measuredLeft = ((*readings)[25].getLocalY() - (*readings)[10].getLocalY())/((*readings)[25].getLocalX() - (*readings)[10].getLocalX());
    float thLeft = atan(measuredLeft)*180/PI;
    float distToRightWall = 0.0, distToLeftWall = 0.0;
    for(int i=0;i<=15;i++){
      distToRightWall += ((*readings)[155+i].getRange())*cos((25-thRight-i)*PI/180)/15.0;
      distToLeftWall += ((*readings)[25-i].getRange())*cos((25-thLeft-i)*PI/180)/15.0;
    }

    float theta = (thLeft+thRight)/2;

    correctionAngle = ((theta/3) * double(fabs(theta) > thThresh)) + (5 * double(distToRightWall < distToRightThresh)) - (5 * double(distToLeftWall < distToLeftThresh));
    robot->setDeltaHeading(correctionAngle);
  }
  else if (canAlignRight(sick) && readings->size() != 0){
    std::cout << "Aligning to Right Line" << "\n";
    float measuredRight = ((*readings)[155].getLocalY() - (*readings)[170].getLocalY())/((*readings)[155].getLocalX() - (*readings)[170].getLocalX());
    float thRight = atan(measuredRight)*180/PI;
    float distToRightWall = 0.0;
    for(int i=0;i<=15;i++){
      distToRightWall += ((*readings)[155+i].getRange())*cos((25-thRight-i)*PI/180)/15.0;
    }
    correctionAngle = ((thRight/3) * double(fabs(thRight) > thThresh)) + (5 * double(distToRightWall < distToRightThresh));
    robot->setDeltaHeading(correctionAngle);   
  }
  else if (canAlignLeft(sick) && readings->size() != 0){
    std::cout << "Aligning to Left Line" << "\n"; 
    float measuredLeft = ((*readings)[25].getLocalY() - (*readings)[10].getLocalY())/((*readings)[25].getLocalX() - (*readings)[10].getLocalX());
    float thLeft = atan(measuredLeft)*180/PI;
    float distToLeftWall = 0.0;
    for(int i=0;i<=15;i++){
      distToLeftWall += ((*readings)[25-i].getRange())*cos((25-thLeft-i)*PI/180)/15.0;
    }
    correctionAngle = ((thLeft/3) * double(fabs(thLeft) > thThresh)) - (5 * double(distToLeftWall < distToLeftThresh));
    robot->setDeltaHeading(correctionAngle);       
  }
}

