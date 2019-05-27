// Library (Compass) robô de futebol 2018 CIC Robotics
// 16/01/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "Compass.h"

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

 extern "C" {
 #include "extApi.h"
 }
#endif

Compass::Compass()
	:compassValue(0),
	clientIDSimulation(0),//Simulation
	compassSimulation(0)//Simulation
{

}

float Compass::value()
{
	#if defined(ARDUINO)
	//return analog(...)
		return 0;
	#else

		float angles[3] = { 0, 0, 0 };

		simxGetObjectOrientation(clientIDSimulation, compassSimulation, -1, (simxFloat*)angles, simx_opmode_oneshot);

		float degreeX = ((angles[2] * 180 / M_PI))*(-1)-270;//+45 for simulation robot compansation(X axis)
		//degreeX -= 270;
		
		while (degreeX < 0)
			degreeX += 360;
		while (degreeX >= 360)
			degreeX -= 360;
		return degreeX;	
	#endif
	//return compassValue;
}

void Compass::update()
{
	//implement this in the future
}

void Compass::initializeSimulation(int ID)
{
	clientIDSimulation = ID;
	#if !defined(ARDUINO)
	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*) "Compass", (simxInt *)&compassSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
	{
		cout << "Compass handle not found!" << endl;
	}
	else
	{
		cout << "Connected to the Compass!" << std::endl;
	}
	#endif
}

