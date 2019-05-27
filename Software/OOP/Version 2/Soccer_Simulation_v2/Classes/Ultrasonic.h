///////////////////////////////////////////////////////////
//  Ultrasonic.h
//  Implementation of the Class Ultrasonic
//  Created on:      21-mar-2018 21:46:59
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "Sensor.h"

class Ultrasonic : public Sensor
{

public:
#if defined(ARDUINO)
	Ultrasonic(int pinEcho, int pinPower);

	void begin();
#else
	Ultrasonic(int _sonarNumber, int robotN);
#endif
	virtual ~Ultrasonic();
	//get property
	virtual float value();
	//simulation
#if !defined(ARDUINO)
	void initializeSimulation(int ID);
#endif
private:
	int sonarNumber;
};
#endif // ULTRASONIC_H
