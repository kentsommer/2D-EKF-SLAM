#include "movementcontroller.h"
#include "Aria.h"

#define PI 3.14159265

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
  float distToFrontWallLeft = 0;
  float distToFrontWallRight = 0;
  bool shouldTurn = false;
  sick->lockDevice();
  std::vector<ArSensorReading> *readings = sick->getRawReadingsAsVector();
  sick->unlockDevice();
  
  //Sample every 2nd reading from 80 + 
  for(int i=0;i<=10;i=i+2){
    if(readings->size() != 0){
      distToFrontWallLeft += fabs(((*readings)[80+i].getRange())*sin((80+i)*PI/180));
    }
  }

  for(int i=0;i<=10;i=i+2){
    if(readings->size() != 0){
      distToFrontWallRight += fabs(((*readings)[90+i].getRange())*sin((90+i)*PI/180));
    }
  }
  distToFrontWallLeft/=5; // Average the distnace
  distToFrontWallRight/=5; //Average the distance
  if (distToFrontWallLeft < 1000 || distToFrontWallRight < 1000){ // 1 Meters, need to figure out what distance is good
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
    float slope1 = getSlope(160, 170, readings);
    float slope2 = getSlope(150, 170, readings);
    float slope3 = getSlope(150, 160, readings);
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
    float slope1 = getSlope(20, 10, readings);
    float slope2 = getSlope(30, 10, readings);
    float slope3 = getSlope(30, 20, readings);
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

  float correctionAngle = 0;
  
  if (canAlignRight(sick) && canAlignLeft(sick) && readings->size() != 0){
    std::cout << "Aligning to Both Lines" << "\n";
    float rightSlope = getSlope(155, 170, readings);
    float leftSlope = getSlope(25, 10, readings);
    float thRight = getTheta(rightSlope);
    float thLeft = getTheta(leftSlope);
    float distToRightWall = 0.0, distToLeftWall = 0.0;
    for(int i=0;i<=15;i++){
      distToRightWall += getDistance(155, 25, thRight, i, readings)/15.0;
      distToLeftWall += getDistance(25, 25, thLeft, i, readings)/15.0;
    }
    float theta = (thLeft+thRight)/2;
    correctionAngle = getCorrectionAngleCombined(theta, distToRightWall, distToLeftWall);
    robot->setDeltaHeading(correctionAngle);
  }
  else if (canAlignRight(sick) && readings->size() != 0){
    std::cout << "Aligning to Right Line" << "\n";
    float rightSlope = getSlope(155, 170, readings);
    float thRight = getTheta(rightSlope);
    float distToRightWall = 0.0;
    for(int i=0;i<=15;i++){
      distToRightWall += getDistance(155, 25, thRight, i, readings)/15.0;
    }
    correctionAngle = getCorrectionAngle(thRight, distToRightWall);
    robot->setDeltaHeading(correctionAngle);   
  }
  else if (canAlignLeft(sick) && readings->size() != 0){
    std::cout << "Aligning to Left Line" << "\n"; 
    float leftSlope = getSlope(25, 10, readings);
    float thLeft = getTheta(leftSlope);
    float distToLeftWall = 0.0;
    for(int i=0;i<=15;i++){
      distToLeftWall += getDistance(25, 25, thLeft, i, readings)/15.0;
    }
    correctionAngle = getCorrectionAngle(thLeft, distToLeftWall);
    robot->setDeltaHeading(correctionAngle);       
  }
}


float getSlope(float angle1, float angle2, std::vector<ArSensorReading> *readings){
  return ((*readings)[angle1].getLocalY() - (*readings)[angle2].getLocalY())/((*readings)[angle1].getLocalX() - (*readings)[angle2].getLocalX());
}

float getDistance(float angle1, float angle2, float theta, int indc, std::vector<ArSensorReading> *readings){
  return ((*readings)[angle1+indc].getRange())*cos((angle2-theta-indc)*PI/180);
}

float getCorrectionAngleCombined(float theta, float distRW, float distLW){
  float threshDist = 970;
  float thThresh = 2;
  return ((theta/3) * double(fabs(theta) > thThresh)) + (5 * double(distRW < threshDist)) - (5 * double(distLW < threshDist));
}

float getCorrectionAngle(float theta, float dist){
  float threshDist = 970;
  float thThresh = 2;
  return ((theta/3) * double(fabs(theta) > thThresh)) + (5 * double(dist < threshDist));
}

float getTheta(float slope){
  return atan(slope)*180/PI;
}