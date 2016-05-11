#ifndef MOVEMENTCONTROLLER_H
#define MOVEMENTCONTROLLER_H

#include <iostream>
#include <unistd.h>
#include <mutex>
#include <vector>
#include "Aria.h"


const int STEP_SLEEP = 1000000;
const int FORWARD_VEL = 300;
const int TURN_VEL = 50;

struct robot_info{
  ArRobot* robot;
  ArSick* sick;
};

void* move_control(void* args);
bool shouldStop(ArSick* sick);
bool canAlignRight(ArSick* sick);
bool canAlignLeft(ArSick* sick);
void alignToWall(ArRobot* robot, ArSick* sick);
float getSlope(float angle1, float angle2, std::vector<ArSensorReading> readings);
float getDistance(float angle1, float angle2, float theta, int indc, std::vector<ArSensorReading> readings);
float getCorrectionAngleCombined(float theta, float distRW, float distLW);
float getCorrectionAngleLeft(float theta, float dist);
float getCorrectionAngleRight(float theta, float dist);
float getTheta(float slope);
bool shouldTurnLeft(ArSick* sick);
void makeLeftTurn(ArRobot* robot, ArSick* sick);


class MovementController
{
public:
  MovementController(ArRobot* robot, ArSick* sick);
  ~MovementController();
  void start();
  void join();
private:
  ArRobot* robot;
  ArSick* laserScanner;
  pthread_t thread;
};

#endif // MOVEMENTCONTROLLER_H
