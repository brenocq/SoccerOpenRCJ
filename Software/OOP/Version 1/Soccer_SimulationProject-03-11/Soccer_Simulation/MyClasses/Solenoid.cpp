// Library (Solenoid) robô de futebol 2018 CIC Robotics
// 11/03/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "Solenoid.h"

#include "TimeThread.h"

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else

#include <string>//string
using std::string;
#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;

extern "C" {
#include "extApi.h"
}
#endif

Solenoid::Solenoid()
	: clientIDSimulation(0), handleSolenoid(0)
{
}

void Solenoid::initializeSimulation(int ID)
{
	clientIDSimulation = ID;

#if !defined(ARDUINO)
	if (simxGetObjectHandle(clientIDSimulation, "Solenoid", &handleSolenoid, simx_opmode_blocking) != simx_return_ok)
		cout << "Solenoid not found!" << endl;
	else
		cout << "Connected to the Solenoid" << endl;
#endif
}

void Solenoid::kick()
{
	extApi_sleepMs(50);
	simxSetJointTargetPosition(clientIDSimulation, handleSolenoid, .04, simx_opmode_oneshot);
	extApi_sleepMs(100);
	simxSetJointTargetPosition(clientIDSimulation, handleSolenoid, 0, simx_opmode_oneshot);
	cout << "!";
}
