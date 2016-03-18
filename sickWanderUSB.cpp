/*
ActivMedia Robotics Interface for Applications (ARIA)
Copyright (C) 2002, ActivMedia Robotics, LLC


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
ActivMedia Robotics for information about a commercial version of ARIA at 
robots@activmedia.com or 
ActivMedia Robotics, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481

*/

#include "Aria.h"

/*
  This program will just have the robot wander around, it uses some avoidance 
  routines, then just has a constant velocity.
*/

int main(int argc, char **argv)
{
	int ret;
  // robot
  ArRobot robot;
  // the laser
  ArSick sick;
  ArSerialConnection laserCon;

  ArSerialConnection con;
  //int ret;
  std::string str;


  // sonar, must be added to the robot
  ArSonarDevice sonar;

  // the actions we'll use to wander
  // recover from stalls
  ArActionStallRecover recover;
  // react to bumpers
  ArActionBumpers bumpers;
  // limiter for close obstacles
  ArActionLimiterForwards limiter("speed limiter near", 300, 600, 250, 1.1);
  // limiter for far away obstacles
  ArActionLimiterForwards limiterFar("speed limiter far", 300, 1100, 600, 1.1);
  // limiter for the table sensors
  ArActionLimiterTableSensor tableLimiter;
  // actually move the robot
  ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
  // turn the orbot if its slowed down
  ArActionTurn turn;

  // mandatory init
  Aria::init();

  // Parse all our args
  ArSimpleConnector connector(&argc, argv);
  connector.parseArgs();
  
  if (argc > 1)
  {
    connector.logOptions();
    exit(1);
  }
  
  // add the sonar to the robot
  robot.addRangeDevice(&sonar);
  // add the laser to the robot
  robot.addRangeDevice(&sick);

  // try to connect, if we fail exit
//  if (!connector.connectRobot(&robot))
//  {
//    printf("Could not connect to robot... exiting\n");
//    Aria::shutdown();
//    return 1;
//  }
// open the connection, just using the defaults, if it fails, exit
printf("got here 1");
  if ((ret = con.open("/dev/ttyUSB0")) != 0)
  {
    str = con.getOpenMessage(ret);
    printf("Open failed: %s\n", str.c_str());
    Aria::shutdown();
    return 1;
  }
  
  printf("got here 2");
  
   // set the robot to use the given connection
  robot.setDeviceConnection(&con);

  // do a blocking connect, if it fails exit
  if (!robot.blockingConnect())
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }

  
  
  
  // turn on the motors, turn off amigobot sounds
  //robot.comInt(ArCommands::SONAR, 0);
  robot.comInt(ArCommands::SOUNDTOG, 0);

  // add the actions
  robot.addAction(&recover, 100);
  robot.addAction(&bumpers, 75);
  robot.addAction(&limiter, 49);
  robot.addAction(&limiterFar, 48);
  robot.addAction(&tableLimiter, 50);
  robot.addAction(&turn, 30);
  robot.addAction(&constantVelocity, 20);
  
  // start the robot running, true so that if we lose connection the run stops
  robot.runAsync(true);

//  connector.setupLaser(&sick);

  // now that we're connected to the robot, connect to the laser
  //sick.runAsync();

  //if (!sick.blockingConnect())
 // {
 //   printf("Could not connect to SICK laser... exiting\n");
 //   Aria::shutdown();
 //   return 1;
 // }
  sick.setDeviceConnection(&laserCon);
  if((ret=laserCon.open("/dev/ttyUSB1")) !=0){
	  Aria::shutdown();
	  return 1;
  }
  sick.configureShort(false, ArSick::BAUD38400, ArSick::DEGREES180, ArSick::INCREMENT_HALF);

  sick.runAsync();
  if(!sick.blockingConnect()){
	  printf("Could not get sick...exiting\n");
	  Aria::shutdown();
	  return 1;
  }
  printf("We are connected to the laser!");
  
  robot.lock();
  robot.comInt(ArCommands::ENABLE, 1);
  robot.unlock();

  robot.waitForRunExit();
  // now exit
  Aria::shutdown();
  return 0;
}
