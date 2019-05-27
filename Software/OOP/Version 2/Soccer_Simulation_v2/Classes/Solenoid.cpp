///////////////////////////////////////////////////////////
//  Solenoid.cpp
//  Implementation of the Class Solenoid
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Solenoid.h"

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
#include "extApi.h"
}
#endif

#if defined(ARDUINO)
Solenoid::Solenoid(int pin1, int pin2)
	:Actuator(pin1, pin2)
{

}
#else
Solenoid::Solenoid(string name, int robotN)
	: Actuator(name, robotN)
{

}
#endif

Solenoid::~Solenoid(){

}

void Solenoid::kick(){
	extApi_sleepMs(50);
	simxSetJointTargetPosition(clientIDSimulation, handleSimulation, .03, simx_opmode_oneshot);
	extApi_sleepMs(100);
	simxSetJointTargetPosition(clientIDSimulation, handleSimulation, 0, simx_opmode_oneshot);
}