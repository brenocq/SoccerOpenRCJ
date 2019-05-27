// Library (Motors) robô de futebol 2018 CIC Robotics
// 05/03/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "Dribble.h"

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
#include "extApi.h"
}
#endif

Dribble::Dribble()
	:maxPower(255), clientIDSimulation(0), dribbleSimulation(-1)
{
	dribble[3] = { 0 };//pins [+,-,pwm]
}

void Dribble::setMaxPower(int maximumPower)
{
	maxPower = maximumPower;
}

void pin(int pos, int neg, int pwm)//			("FR"/"FL"/"BR"/"BL", pin+, pin-, pinPWM)
{
#if defined(ARDUINO)
	dribble[0] = pos;
	dribble[1] = neg;
	dribble[2] = pwm;

	}
#else
	cout << "Dribble pins: you can set the motor pins!!" << endl;
#endif
}

void Dribble::initializeSimulation(int ID)
{
#if !defined(ARDUINO)
	clientIDSimulation = ID;
	//--------------Motor FR(front-right)
	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*) "Dribble", (simxInt *)&dribbleSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << "Dribble handle not found!" << std::endl;
	else
		cout << "Connected to the Dribble!" << std::endl;
	
	//-----------------------------Velocity 0 in motors
	motor(0);
#endif
}

void Dribble::motor(float motorPower)
{
#if defined(ARDUINO)

#else

	float simulationPower = (motorPower / 255) * 700 * M_PI / 180;
	simxSetJointTargetVelocity(clientIDSimulation, dribbleSimulation, (simxFloat)simulationPower, simx_opmode_streaming);
		
#endif
}