///////////////////////////////////////////////////////////
//  Ultrasonic.cpp
//  Implementation of the Class Ultrasonic
//  Created on:      21-mar-2018 21:46:59
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Ultrasonic.h"

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
Ultrasonic::Ultrasonic(int pinEcho, int pinPower)//pins[0]=echo PWM  pins[1]=Power
	:Sensor(pinEcho, pinPower)
{

}

void Ultrasonic::begin(){
	pinMode(pins[1], OUTPUT);                   // Power pin
	digitalWrite(pins[1], LOW);                // Sensor off
	pinMode(pins[0], INPUT);                    // Echo pin
}

#else
Ultrasonic::Ultrasonic(int _sonarNumber, int robotN)
	: Sensor("", robotN), sonarNumber(_sonarNumber)
{

}
#endif

Ultrasonic::~Ultrasonic(){

}


float Ultrasonic::value(){
#if defined(ARDUINO)
	digitalWrite(pins[1], HIGH);
	delay(800);
	unsigned long LowLevelTime = pulseIn(pins[0], LOW);
	delay(100);
	digitalWrite(pins[1], LOW);
	if (LowLevelTime >= 45000)                 // the reading is invalid.
	{
		sonarNumber = 0;
		return 0;
	}
	else{
		sonarNumber = LowLevelTime / 50;
		return sonarNumber;   // every 50us low level stands for 1cm
	}
#else
	simxUChar state;
	simxFloat coord[3];

	if (simxReadProximitySensor(clientIDSimulation, handleSimulation, &state, coord, NULL, NULL, simx_opmode_streaming) == simx_return_ok)
	{
		float dist = coord[2];
		if (state > 0)
		{
			//sonar 3,6 and 9 are inside the robot(ratio 10)
			return (sonarNumber % 3) == 0 ? dist * 100 + 10 : dist * 100 + 11;// 11 is the ratio of the robot
		}
	}
	return 0;
#endif
}

#if !defined(ARDUINO)
void Ultrasonic::initializeSimulation(int ID){
	clientIDSimulation = ID;

	string name = "Sonar" + std::to_string(sonarNumber) + "#" + std::to_string(robotNumber);

	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)name.c_str(), (simxInt *)&handleSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << "Sonar" << sonarNumber << "- robot" << robotNumber << " not found!" << std::endl;
	else
	{
		cout << "Connected to the sensor " << "Sonar" << sonarNumber << "- robot" << robotNumber << std::endl;
		simxReadProximitySensor(clientIDSimulation, handleSimulation, NULL, NULL, NULL, NULL, simx_opmode_streaming);
	}
		
}
#endif