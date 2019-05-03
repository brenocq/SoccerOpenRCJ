///////////////////////////////////////////////////////////
//  Servo.h
//  Implementation of the Class Servo
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef SERVO_H
#define SERVO_H

#include "Actuator.h"
#include "TimeThread.h"

#if !defined(ARDUINO)
#include <string>//string
using std::string;
#endif

class Servo : public Actuator
{

public:
#if defined(ARDUINO)
	Servo(int pin);
#else
	Servo(string name, int robotN);
#endif
	virtual ~Servo();

	void setAngle(int _angle);
	int getAngle() const;

private:
	int angle;
	TimeThread *smoothAngle;

};
#endif // SERVO_H
