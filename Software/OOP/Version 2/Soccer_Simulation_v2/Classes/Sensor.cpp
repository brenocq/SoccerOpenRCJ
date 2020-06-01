///////////////////////////////////////////////////////////
//  Sensor.cpp
//  Implementation of the Class Sensor
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Sensor.h"

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
#include "../CopelliaSim/extApi.h"
}
#endif

#if defined(ARDUINO)
Sensor::Sensor(int pin1, int pin2)
{
	pins[0]=pin1;
	pins[1]=pin2;
}
#else
Sensor::Sensor(string name, int robotN)
	: sensorName(name), lastValue(0)
{
	setRobotNumber(robotN);
}
#endif

Sensor::~Sensor(){

}

//get property
int Sensor::getLastvalue() const
{
	return lastValue;
}
//simulation
#if !defined(ARDUINO)
void Sensor::setRobotNumber(int number){
	robotNumber = number;
}
void Sensor::initializeSimulation(int ID){
	clientIDSimulation = ID;

	string name = sensorName + "#" + std::to_string(robotNumber);

	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)name.c_str(), (simxInt *)&handleSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << sensorName << "- robot" << robotNumber << " not found!" << std::endl;
	else
		cout << "Connected to the sensor " << sensorName << "- robot" << robotNumber << std::endl;
}
#endif

