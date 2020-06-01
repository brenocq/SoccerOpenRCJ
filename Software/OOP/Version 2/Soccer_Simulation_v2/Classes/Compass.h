///////////////////////////////////////////////////////////
//  Compass.h
//  Implementation of the Class Compass
//  Created on:      21-mar-2018 21:46:56
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef COMPASS_H
#define COMPASS_H

#include "Sensor.h"

#if defined(ARDUINO)//arduino libraries

#if ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif ARDUINO<100
#include "WProgram.h"//arduino old version
#endif
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

class Compass : public Sensor
{

public:
#if defined(ARDUINO)
	Compass();

	void begin();
	void print(bool plotter=false);
	void updateAccel();
	int accelX();
	int accelY();
	int accelZ();
#else
	Compass(string name, int robotN);
#endif

	virtual ~Compass();

	//value
	virtual float value();

private:
	bool isFirstReading; // ajust the first angle to 0
	float adjustFront;
	int accelerometerX;
	int accelerometerY;
	int accelerometerZ;
};
#endif // COMPASS_H
