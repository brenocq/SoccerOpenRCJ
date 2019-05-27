///////////////////////////////////////////////////////////
//  Solenoid.h
//  Implementation of the Class Solenoid
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef SOLENOID_H
#define SOLENOID_H

#include "Actuator.h"

class Solenoid : public Actuator
{

public:
#if defined(ARDUINO)
	Solenoid(int pin1, int pin2);
#else
	Solenoid(string name, int robotN);
#endif
	virtual ~Solenoid();
	
	void kick();

};
#endif // SOLENOID_H
