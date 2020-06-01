///////////////////////////////////////////////////////////
//  Servo.cpp
//  Implementation of the Class Servo
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Servo.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

#if !defined(ARDUINO)
extern "C" {
#include "../CopelliaSim/extApi.h"
}
#endif

#if defined(ARDUINO)
Servo::Servo(int pin)
	: Actuator(pin, 0)
{

}
#else
Servo::Servo(string name, int robotN)
	: Actuator(name, robotN), angle(60)
{
	smoothAngle = new TimeThread(100);
}
#endif


Servo::~Servo(){

}

void Servo::setAngle(int _angle){
	
	if (smoothAngle->executeThread())
	{
		_angle > angle ? angle+=4: angle;
		_angle < angle ? angle-=4: angle;
	}

	if (abs(_angle - angle) <= 5){
		_angle > angle ? angle ++ : angle;
		_angle < angle ? angle -- : angle;
	}

	simxSetJointPosition(clientIDSimulation, handleSimulation, static_cast<float>(angle)*M_PI / 180, simx_opmode_streaming);
}


int Servo::getAngle()const {
	return angle;
}