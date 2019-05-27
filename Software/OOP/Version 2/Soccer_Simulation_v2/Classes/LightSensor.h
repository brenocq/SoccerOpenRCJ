///////////////////////////////////////////////////////////
//  LightSensor.h
//  Implementation of the Class LightSensor
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef LIGHTSENSOR_H
#define LIGHTSENSOR_H

#include "Sensor.h"

class LightSensor : public Sensor
{

public:
#if defined(ARDUINO)
	LightSensor(int pin);
#else
	LightSensor(string name, int robotN);
#endif
	virtual ~LightSensor();

	virtual float value();

	void initializeSimulation(int ID);

private:

#if !defined(ARDUINO)
	int handleBPSensor;
	int handleBPCollision;
	int handleBPBall;
	bool isCollisionSensor;
	bool robotWithBall;
#endif

};
#endif // LIGHTSENSOR_H
