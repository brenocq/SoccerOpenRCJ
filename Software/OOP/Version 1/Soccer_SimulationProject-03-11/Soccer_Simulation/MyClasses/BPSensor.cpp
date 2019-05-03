// Library (Ball Possession Sensor) robô de futebol 2018 CIC Robotics
// 11/03/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "BPSensor.h"

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
#include <iomanip>//cout,cin
using std::setprecision;

extern "C" {
#include "extApi.h"
}
#endif

BPSensor::BPSensor()
	: clientIDSimulation(0), handleBPCollision(0), handleBPSensor(0), handleBall(0),
	robotWithBall(false)
{
	timeWithoutBall = new TimeThread(500);
}

void BPSensor::initializeSimulation(int ID)
{
	clientIDSimulation = ID;

#if !defined(ARDUINO)
	if (simxGetObjectHandle(clientIDSimulation, "BPsensor", &handleBPSensor, simx_opmode_blocking) != simx_return_ok)
		cout << "BPsensor not found!" << endl;
	else
		cout << "Connected to the BPsensor " << endl;

	if (simxGetCollisionHandle(clientIDSimulation, "BPsensorCollision", &handleBPCollision, simx_opmode_blocking) != simx_return_ok)
		cout << "BPsensorCollision not found!" << endl;
	else
		cout << "Connected to the BPsensorCollision " << endl;

	if (simxGetObjectHandle(clientIDSimulation, "Ball", &handleBall, simx_opmode_blocking) != simx_return_ok)
		cout << "Ball not found!" << endl;
	else
		cout << "Connected to the Ball" << endl;

#endif
}

bool BPSensor::value()
{
#if defined(ARDUINO)

#else
	updateSimulationBall();
	char state[3];
	if (simxReadCollision(clientIDSimulation, handleBPCollision, (simxUChar*)state, simx_opmode_streaming) == simx_return_ok)
	{
		if (state[0] != 0)
		{
			if (timeWithoutBall->executePointThread() == true)
			{
				robotWithBall = true;
			}
			return true;
		}
		else
			return false;
	}

#endif
}

void BPSensor::updateSimulationBall()
{
#if !defined(ARDUINO)

	if (robotWithBall == true)
	{
		float ballPos[3] = { 0, 0.023, 0 };
		simxSetObjectPosition(clientIDSimulation, handleBall, handleBPSensor, ballPos, simx_opmode_oneshot);
	}


#endif
}

void BPSensor::setRobotWithBall(bool b)
{
	if (b == false)
	{
		if (robotWithBall == true)
		{
			float ballPos[3] = { 0, 0.05, 0 };
			simxSetObjectPosition(clientIDSimulation, handleBall, handleBPSensor, ballPos, simx_opmode_oneshot);

			robotWithBall = false;
			timeWithoutBall->setPointTime();
		}
	}
	else
	{
		robotWithBall = true;
	}
}