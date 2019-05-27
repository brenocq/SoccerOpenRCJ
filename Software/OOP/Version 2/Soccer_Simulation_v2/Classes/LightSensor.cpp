///////////////////////////////////////////////////////////
//  LightSensor.cpp
//  Implementation of the Class LightSensor
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "LightSensor.h"

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else

#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;
#include <iomanip>//setprecision
using std::setprecision;

extern "C" {
#include "extApi.h"
}
#endif

#if defined(ARDUINO)
LightSensor::LightSensor(int pin)
	:Sensor(pin, 0)
{

}
#else
LightSensor::LightSensor(string name, int robotN)
	: Sensor(name, robotN), isCollisionSensor(false)
{

}
#endif

LightSensor::~LightSensor(){

}

float LightSensor::value(){
#if defined(ARDUINO)

#else
	if (isCollisionSensor == true)//If is a collision sensor in simulation
	{
		char state[3];
		if (simxReadCollision(clientIDSimulation, handleBPCollision, (simxUChar*)state, simx_opmode_streaming) == simx_return_ok)
		{
			if (state[0] != 0)
			{
				return 0;
			}
			else
				return 100;
		}
	}
#endif
}

void LightSensor::initializeSimulation(int ID)
{
	clientIDSimulation = ID;

#if !defined(ARDUINO)
	string name = sensorName + "#" + std::to_string(robotNumber);

	string collisionName = "LS_BallPos#" + std::to_string(robotNumber);
	if (name == collisionName)//Detect if is a collision sensor in simulation
	{
		isCollisionSensor = true;
		if (simxGetObjectHandle(clientIDSimulation, collisionName.c_str(), &handleBPSensor, simx_opmode_blocking) != simx_return_ok)
			cout << collisionName << " not found!" << endl;
		else
			cout << "Connected to the" << collisionName << endl;

		string collisionDetection = "LS_BallPosCollision" + std::to_string(robotNumber);

		if (simxGetCollisionHandle(clientIDSimulation, collisionDetection.c_str(), &handleBPCollision, simx_opmode_blocking) != simx_return_ok)
			cout << collisionDetection << " not found!" << endl;
		else
			cout << "Connected to the " << collisionDetection << endl;

		if (simxGetObjectHandle(clientIDSimulation, "Ball", &handleBPBall, simx_opmode_blocking) != simx_return_ok)
			cout << "Ball not found!" << endl;
		else
			cout << "Connected to the Ball" << endl;
	}

#endif
}