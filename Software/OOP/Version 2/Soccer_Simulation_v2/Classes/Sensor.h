///////////////////////////////////////////////////////////
//  Sensor.h
//  Implementation of the Class Sensor
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef SENSOR_H
#define SENSOR_H

#if !defined(ARDUINO)

#include <string>//string
using std::string;
#endif

class Sensor
{

public:
#if defined(ARDUINO)
	Sensor(int pin1,int pin2);
#else
	Sensor(string name, int robotN);
#endif
	virtual ~Sensor();

	//get property
	virtual float value() = 0;
	int getLastvalue() const;
#if !defined(ARDUINO)
	//set property
	void setRobotNumber(int number);
	//simulation
	void initializeSimulation(int ID);
#endif
protected:
	float lastValue;
#if defined(ARDUINO)
	int pins[2];
#else
	int clientIDSimulation;
	int handleSimulation;
	int robotNumber;
	string sensorName;
#endif

};
#endif // SENSOR_H
