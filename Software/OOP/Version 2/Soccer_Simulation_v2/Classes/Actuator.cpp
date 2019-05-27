///////////////////////////////////////////////////////////
//  Actuator.cpp
//  Implementation of the Class Actuator
//  Created on:      21-mar-2018 21:46:55
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Actuator.h"

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
Actuator::Actuator(int pin1, int pin2)
{
	pins[0]=pin1;
	pins[1]=pin2;
}
#else
Actuator::Actuator(string name, int robotN)
	: motorName(name),robotNumber(robotN)
{
	setRobotNumber(robotN);
}
#endif


Actuator::~Actuator(){

}

//simulation
#if !defined(ARDUINO)
void Actuator::setRobotNumber(int number){
	robotNumber = number;
}
void Actuator::initializeSimulation(int ID){
	clientIDSimulation = ID;

	string name = "Motor" + motorName + "#" +std::to_string(robotNumber);

	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)name.c_str(), (simxInt *)&handleSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << name << "- robot" << robotNumber << " not found!" << std::endl;
		else
			cout << "Connected to the motor  " << motorName << "- robot" << robotNumber << std::endl;
}
#endif