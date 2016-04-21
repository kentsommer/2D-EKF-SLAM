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
void move_forward(ArRobot* robot);
void move_backward(ArRobot* robot);
void turn_left(ArRobot* robot);
void turn_right(ArRobot* robot);
void stop(ArRobot* robot);
double getClosestReading(ArSick* sick);
bool shouldTurn(ArSick* sick);
bool canAlignRight(ArSick* sick);
bool canAlignLeft(ArSick* sick);
void alignToWall(ArRobot* robot, ArSick* sick);


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
