// Code to obtain readings from the SICK laser scanner

// how to run this code? ./sickReadings -rp <robotport> -lp <laserport>


#include "Aria.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>

using namespace std;


ArRobot robot;
ArSick sick;



int main(int argc, char **argv)
{

	int t, cnt;
	double laser_dist[900];
	double laser_angle[900];
	std::list<ArSensorReading *> *readings;
	std::list<ArSensorReading *>::iterator it;


	ArKeyHandler keyHandler;

	Aria::init();

	// Add the key handler to Aria so other things can find it
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);

	// add the laser to the robot
	robot.addRangeDevice(&sick);

	// Parse all our args
	ArSimpleConnector connector(&argc, argv);

	if (!connector.parseArgs() || argc > 1)
	{
		connector.logOptions();
		exit(1);
	}

	robot.addRangeDevice(&sick);

	// try to connect, if we fail exit
	if (!connector.connectRobot(&robot))
	{
		printf("Could not connect to robot... exiting\n");
		Aria::shutdown();
		return 1;
	}


	// start the robot running, true so that if we lose connection the run stops
	robot.runAsync(true);

	// now set up the laser
	sick.configureShort(false,ArSick::BAUD38400,ArSick::DEGREES180,ArSick::INCREMENT_ONE);

	connector.setupLaser(&sick);

	sick.runAsync();

	if (!sick.blockingConnect())
	{
		printf("Could not connect to SICK laser... exiting\n");
		Aria::shutdown();
		return 1;
	}
        
	while(1)
	{

		readings=(list<ArSensorReading *,allocator<ArSensorReading *> > *)sick.getRawReadings();//CurrentBuffer..
		while (readings==NULL) 	readings=(list<ArSensorReading *,allocator<ArSensorReading *> > *)sick.getRawReadings();



			t=0;
			for(it=readings->begin();it!=readings->end(); it++)
			{
				laser_dist[t]=(*it)->getRange();
				laser_angle[t]=-90+t;
				//cout << "laser angle: " << laser_angle[t] << " laser dist.: " << laser_dist[t] <<" "<<"\n";
				t++;
			}
			cnt++;

		for (t=0; t<181; t++){
        	cout << "laser angle: " << laser_angle[t] << " laser dist.: " << laser_dist[t] <<" "<<"\n";
        }
						
	} //end while 1<2
	
        
	robot.waitForRunExit();
	Aria::shutdown();
	return 0;
}