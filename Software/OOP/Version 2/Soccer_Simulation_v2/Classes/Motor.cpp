///////////////////////////////////////////////////////////
//  Motor.cpp
//  Implementation of the Class Motor
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Motor.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

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
Motor::Motor(int pin1, int pin2, int _pinPWM)
	:Actuator(pin1, pin2), maxPower(255), pinPWM(_pinPWM)
{
	pinMode(pins[0], OUTPUT);
	pinMode(pins[1], OUTPUT);
	pinMode(pinPWM, OUTPUT);


}
#else
Motor::Motor(string name, int robotN)
	: Actuator(name, robotN)
{
}
#endif

Motor::~Motor(){

}

void Motor::power(float value){
	value>255 ? value=255 :value=value;
	value<(-255) ? value=-255 :value=value;

	//value *= (maxPower / 255);
	//cout<<value;
#if defined(ARDUINO)
	if (value > 0){
		digitalWrite(pins[0], HIGH);
		digitalWrite(pins[1], LOW);
		analogWrite(pinPWM, value);
	}
	else{
		digitalWrite(pins[0], LOW);
		digitalWrite(pins[1], HIGH);
		analogWrite(pinPWM, -value);
	}
#else
	float simulationPower = (value / 255) * 600 * M_PI / 180;

	simxSetJointTargetVelocity(clientIDSimulation, handleSimulation, 
		(simxFloat)simulationPower, simx_opmode_oneshot);
#endif
}