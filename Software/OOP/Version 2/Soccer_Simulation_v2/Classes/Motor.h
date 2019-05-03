///////////////////////////////////////////////////////////
//  Motor.h
//  Implementation of the Class Motor
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef MOTOR_H
#define MOTOR_H

#include "Actuator.h"


#if !defined(ARDUINO)

#include <string>//string
using std::string;
#endif

class Motor : public Actuator
{

public://------------------------------------------------------//public//
#if defined(ARDUINO)
	Motor(int pin1, int pin2, int _pinPWM);
#else
	Motor(string name, int robotN);
#endif
	virtual ~Motor();
	
	void setMaxPower(int value){ maxPower = value; };
	int getMaxPower() const{ return maxPower; };
	void power(float value);
private://----------------------------------------------------//private//
#if defined(ARDUINO)
	int pinPWM;
#endif
	int maxPower;
};
#endif // MOTOR_H
