// Library (Ball Possession Sensor) robô de futebol 2018 CIC Robotics
// 11/03/2018
// Desenvolvedor: Breno Cunha Queiroz

#ifndef BPSENSOR_H
#define BPSENSOR_H

#include "TimeThread.h"

class BPSensor
{
public:
	BPSensor();
	void pins();
	bool value();

	void initializeSimulation(int ID);//initialize the simulation BPsensor
	void updateSimulationBall();

	void setRobotWithBall(bool b);
	
private:
	
	//Simulation
	bool robotWithBall;
	int clientIDSimulation;
	int handleBPCollision;
	int handleBPSensor;
	int handleBall;
	//Objects
	TimeThread *timeWithoutBall;//used in simulation to drop the ball

};

#endif // BPSENSOR_H