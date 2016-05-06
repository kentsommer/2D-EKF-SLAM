#include "movementcontroller.h"
#include "Aria.h"

#define PI 3.14159265

std::vector<ArSensorReading> *rReal;
std::vector<ArSensorReading> readings;

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

// MAIN FUNCTION //
void* move_control(void* args){
  struct robot_info* info;
  info = (struct robot_info*)args;
  info->robot->lock();
  info->robot->setVel(300);
  info->robot->unlock();

  while(1){
    //Get sensor readings 
    info->sick->lockDevice();
    rReal = info->sick->getRawReadingsAsVector();
    std::vector<ArSensorReading>  temp(*rReal);
    readings = temp;
    info->sick->unlockDevice();

    //Begin control loop
    if(shouldTurnLeft(info->sick)){
      //Manage turning left
      info->robot->lock();
      info->robot->setVel(300);
      info->robot->unlock();
      usleep(1000);
      makeLeftTurn(info->robot, info->sick);
      continue;
    }
    else if(shouldStop(info->sick)){
      //Manage Stopping
      stop(info->robot);
    }
    else{
      //Continue forward
      info->robot->lock();
      info->robot->setVel(300);
      info->robot->unlock();
    }
    //Align to walls
    alignToWall(info->robot, info->sick);

    //Rinse and repeat! 
    usleep(1000);
  }

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


bool shouldStop(ArSick* sick){
  // Average distance in front left and front right window of scanner
    // if distance < thresh -> return true
  float distToFrontWallLeft = 0;
  float distToFrontWallRight = 0;
  bool shouldStop = false;


  if(readings.size() == 0){ return false; }
  
  //Sample every 2nd reading from 80 + 
  for(int i=0;i<=10;i=i+2){
    if(readings.size() != 0){
      distToFrontWallLeft += fabs(((readings)[80+i].getRange())*sin((80+i)*PI/180));
    }
  }

  for(int i=0;i<=10;i=i+2){
    if(readings.size() != 0){
      distToFrontWallRight += fabs(((readings)[90+i].getRange())*sin((90+i)*PI/180));
    }
  }
  distToFrontWallLeft/=5; // Average the distnace
  distToFrontWallRight/=5; //Average the distance
  if (distToFrontWallLeft < 1000 || distToFrontWallRight < 1000){ // 1 Meters, need to figure out what distance is good
      shouldStop = true;
  }
  return shouldStop; 
}

bool shouldTurnLeft(ArSick* sick){
  //Get average distance between 0 degrees and 40, if larger than thresh -> return true
  float distToLeftWall = 0;
  float threshDistToLeftWall = 1300;
  bool shouldTurn = false;


  if(readings.size() == 0){ return false; }
  int count = 0;
  for (double theta=0.0;theta<40.0; theta+=5.0){
    if(readings.size() != 0){
        for (int i=0;i<5; i++){
          count = count + 1;
          distToLeftWall += fabs(((readings)[theta+i].getRange())*sin((theta+i)*PI/180));
        }
    }
  }
  distToLeftWall/=count; // Average the distnace
  if (distToLeftWall > threshDistToLeftWall){
      shouldTurn = true;
  }
  return shouldTurn; 
}

void makeLeftTurn(ArRobot* robot, ArSick* sick)
{ 
  robot->lock();
  robot->setRotVel(15.0);
  robot->unlock();

  robot->lock();
  robot->setVel(250);
  robot->unlock();

  usleep(1000);
}


bool canAlignRight(ArSick* sick){
  if(readings.size() != 0){
    // Check slopes between 150:160, 160:170, 150:170 -> if standard deviation is small, use as line.
    float slope1 = getSlope(160, 170, readings);
    float slope2 = getSlope(150, 170, readings);
    float slope3 = getSlope(150, 160, readings);
    float Ex2 = (slope1*slope1+slope2*slope2+slope3*slope3)/3;
    float Ex = (slope1+slope2+slope3)/3;
    float stdDeviation = sqrt(Ex2 - Ex*Ex);
    if (stdDeviation < 0.025){
      return true;
    }
  }
  return false;
}


bool canAlignLeft(ArSick* sick){
  if(readings.size() != 0){
    // Check slopes between 10:20, 20:30, 10:30 -> if standard deviation is small, use as line. 
    float slope1 = getSlope(20, 10, readings);
    float slope2 = getSlope(30, 10, readings);
    float slope3 = getSlope(30, 20, readings);
    float Ex2 = (slope1*slope1+slope2*slope2+slope3*slope3)/3;
    float Ex = (slope1+slope2+slope3)/3;
    float stdDeviation = sqrt(Ex2 - Ex*Ex);
    if (stdDeviation < 0.025){
      return true;
    }
  }
  return false;
}


void alignToWall(ArRobot* robot, ArSick* sick){

  float wallDistThresh = 1400;
  float correctionAngle = 0;
  
  //Check if can find wall on left and right, if so -> center -> align
  if (canAlignRight(sick) && canAlignLeft(sick) && readings.size() != 0){
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

    //Centering
    if(distToRightWall > distToLeftWall){
      robot->setRotVel(-5.0);
    }
    if(distToLeftWall > distToRightWall){
      robot->setRotVel(5.0);
    }
    //Alignment 
    else{
      robot->setDeltaHeading(correctionAngle);
    }
  }

  //Check if can find wall on right, if so -> maintain distance to/from wall -> align
  else if (canAlignRight(sick) && readings.size() != 0){
    float rightSlope = getSlope(155, 170, readings);
    float thRight = getTheta(rightSlope);
    float distToRightWall = 0.0;
    for(int i=0;i<=15;i++){
      distToRightWall += getDistance(155, 25, thRight, i, readings)/15.0;
    }
    correctionAngle = getCorrectionAngleRight(thRight, distToRightWall);

    //Maintaining distance
    if(distToRightWall < wallDistThresh){ 
      robot->setRotVel(5.0);
    }
    else if(distToRightWall > wallDistThresh + 100){
      robot->setRotVel(-5.0);
    }
    //Alignment
    else{
      robot->setDeltaHeading(correctionAngle);
    }  
  }

  //Check if can find wall on left, if so -> maintain distance to/from wall -> align
  else if (canAlignLeft(sick) && readings.size() != 0){
    float leftSlope = getSlope(25, 10, readings);
    float thLeft = getTheta(leftSlope);
    float distToLeftWall = 0.0;
    for(int i=0;i<=15;i++){
      distToLeftWall += getDistance(25, 25, thLeft, i, readings)/15.0;
    }
    correctionAngle = getCorrectionAngleLeft(thLeft, distToLeftWall);

    //Maintaining distance
    if(distToLeftWall < wallDistThresh){
      robot->setRotVel(-5.0); 
    }
    else if(distToLeftWall > wallDistThresh + 100){
      robot->setRotVel(5.0);
    }
    //Alignment
    else{
      robot->setDeltaHeading(correctionAngle);
    }     
  }
}


//Helper functions for Alignment
float getSlope(float angle1, float angle2, std::vector<ArSensorReading> readings){
  return ((readings)[angle1].getLocalY() - (readings)[angle2].getLocalY())/((readings)[angle1].getLocalX() - (readings)[angle2].getLocalX());
}

float getDistance(float angle1, float angle2, float theta, int indc, std::vector<ArSensorReading> readings){
  return ((readings)[angle1+indc].getRange())*cos((angle2-theta-indc)*PI/180);
}

float getCorrectionAngleCombined(float theta, float distRW, float distLW){
  float threshDist = 970;
  float thThresh = 2;
  return ((theta/3) * double(fabs(theta) > thThresh)) + (5 * double(distRW < threshDist)) - (5 * double(distLW < threshDist));
}

float getCorrectionAngleRight(float theta, float dist){
  float threshDist = 970;
  float thThresh = 2;
  return ((theta/3) * double(fabs(theta) > thThresh)) + (5 * double(dist < threshDist));
}

float getCorrectionAngleLeft(float theta, float dist){
  float threshDist = 970;
  float thThresh = 2;
  return ((theta/3) * double(fabs(theta) > thThresh)) + (5 * double(dist < threshDist));
}

float getTheta(float slope){
  return atan(slope)*180/PI;
}