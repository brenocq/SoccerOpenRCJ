///////////////////////////////////////////////////////////
//  Actuator.h
//  Implementation of the Class Actuator
//  Created on:      21-mar-2018 21:46:55
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef ACTUATOR_H
#define ACTUATOR_H

#if !defined(ARDUINO)

#include <string>//string
using std::string;
#endif

class Actuator
{

public://------------------------------------------------------//public//
#if defined(ARDUINO)
	Actuator(int pin1, int pin2);
#else
	Actuator(string name, int robotN);
#endif

	virtual ~Actuator();

	//simulation
#if !defined(ARDUINO)
	void initializeSimulation(int ID);
	void setRobotNumber(int number);
#endif

protected://----------------------------------------------------//protected//
#if defined(ARDUINO)
	int pins[2];
#else
	int clientIDSimulation;
	int handleSimulation;
	int robotNumber;
	string motorName;
#endif

};
#endif // ACTUATOR_H
