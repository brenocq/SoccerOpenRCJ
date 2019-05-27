// Library (Motors) robô de futebol 2018 CIC Robotics
// 16/01/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "Motors.h"
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
#include <iomanip>//setprecision
using std::setprecision;

extern "C" {
#include "extApi.h"
}
#endif

Motors::Motors(Compass &robotCompass)
	: compass(robotCompass)
{
	maxPower = 255;
	Kp = 0.025;
	motorFR[3] = { 0 };//pins [+,-,pwm]
	motorBR[3] = { 0 };//pins [+,-,pwm]
	motorBL[3] = { 0 };//pins [+,-,pwm]
	motorFL[3] = { 0 };//pins [+,-,pwm]
	//Simulation
	clientIDSimulation = 0;
	MotorFRSimulation = -1;
	MotorFLSimulation = -1;
	MotorBRSimulation = -1;
	MotorBLSimulation = -1;
}

void Motors::setMaxPower(int maximumPower)
{
	maxPower = maximumPower;
}

void Motors::pin(MotorInitials motorInitials, int pos, int neg, int pwm)
{
#if defined(ARDUINO)
	switch (motorInitials)
	{
	case FR:
		motorFR[0] = pos;
		motorFR[1] = neg;
		motorFR[2] = pwm;
		break;
	case BR:
		motorBR[0] = pos;
		motorBR[1] = neg;
		motorBR[2] = pwm;
		break;
	case BL:
		motorBL[0] = pos;
		motorBL[1] = neg;
		motorBL[2] = pwm;
		break;
	case FL:
		motorFL[0] = pos;
		motorFL[1] = neg;
		motorFL[2] = pwm;
		break;
	}
#else
	cout << "Motor pins: you can set the motors pins!!" << endl;
#endif
}

void Motors::initializeSimulation(int ID)
{
#if !defined(ARDUINO)
	clientIDSimulation = ID;

	//--------------Motor FR(front-right)
	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*) "Motor_FR", (simxInt *)&MotorFRSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << "motor FR handle not found!" << std::endl;
	else
		cout << "Connected to the motor FR!" << std::endl;
	//--------------Motor BR(back-right)
	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*) "Motor_BR", (simxInt *)&MotorBRSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << "motor BR handle not found!" << std::endl;
	else
		cout << "Connected to the motor BR!" << std::endl;
	//--------------Motor BL(back-left)
	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*) "Motor_BL", (simxInt *)&MotorBLSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << "motor BL handle not found!" << std::endl;
	else
		cout << "Connected to the motor BL!" << std::endl;
	//--------------Motor FL(front-left)
	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*) "Motor_FL", (simxInt *)&MotorFLSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << "motor FL handle not found!" << std::endl;
	else
		cout << "Connected to the motor FL!" << std::endl;
	//-----------------------------Velocity 0 in motors
	motor(FR, 0);
	motor(BR, 0);
	motor(BL, 0);
	motor(FL, 0);
#endif
}

void Motors::relativeMotor(MotorInitials motorInitials, int motorPower)
{
	int spinTimes = 0;//Number of motor spins|EX: spin=2 -> Sonar 1 turns 3/Sonar 2 turns 4
	int motorNumber = 0;

	switch (motorInitials)
	{
	case FR:
		motorNumber = 1;
		break;
	case BR:
		motorNumber = 2;
		break;
	case BL:
		motorNumber = 3;
		break;
	case FL:
		motorNumber = 4;
		break;
	}

	if (compass.value() > 180)
		spinTimes = 4;

	int spinTime_auxiliary = 0;

	if (compass.value() <= 180)
	{
		while (compass.value() - 45 - (90 * spinTime_auxiliary) >= 0)
		{
			spinTimes++;
			spinTime_auxiliary++;
		}
	}
	else
	{
		while ((360 - compass.value()) - 45 - (90 * spinTime_auxiliary) >= 0)
		{
			spinTimes--;
			spinTime_auxiliary++;
		}
	}

	if (spinTimes < 0)
		spinTimes = 4 + spinTimes;

	int RealNumberMotor;//The real number after degree tratament

	RealNumberMotor = motorNumber - spinTimes;
	while (RealNumberMotor > 4)
		RealNumberMotor = RealNumberMotor - 4;
	while (RealNumberMotor < 1)
		RealNumberMotor = RealNumberMotor + 4;

	switch (RealNumberMotor)
	{
	case 1:
		if (spinTimes == 2 || spinTimes == 3)
			motorPower *= (-1);
		motor(FR, motorPower);
		break;
	case 2:
		if (spinTimes == 1 || spinTimes == 2)
			motorPower *= (-1);
		motor(BR, motorPower);
		break;
	case 3:
		if (spinTimes == 2 || spinTimes == 3)
			motorPower *= (-1);
		motor(BL, motorPower);
		break;
	case 4:
		if (spinTimes == 1 || spinTimes == 2)
			motorPower *= (-1);
		motor(FL, motorPower);
		break;
	}
	//cout << "Motor[" << numberMotor << "]= " << RealNumberMotor << "	" << velocity << endl;
}

void Motors::motor(MotorInitials initials, float motorPower)
{
#if defined(ARDUINO)
	switch (initials)
	{
	case FR:
		break;
	case BR:
		break;
	case BL:
		break;
	case FL:
		break;
	}
#else

	float simulationPower = (motorPower / 255) * 700 * M_PI / 180;
	switch (initials)
	{
	case FR:
		simxSetJointTargetVelocity(clientIDSimulation, MotorFRSimulation, (simxFloat)simulationPower, simx_opmode_streaming);
		break;
	case BR:
		simxSetJointTargetVelocity(clientIDSimulation, MotorBRSimulation, (simxFloat)simulationPower, simx_opmode_streaming);
		break;
	case BL:
		simxSetJointTargetVelocity(clientIDSimulation, MotorBLSimulation, (simxFloat)simulationPower, simx_opmode_streaming);
		break;
	case FL:
		simxSetJointTargetVelocity(clientIDSimulation, MotorFLSimulation, (simxFloat)simulationPower, simx_opmode_streaming);
		break;
	}
#endif
}

float Motors::PIDController(int realRobotAngle)
{
	float error;
	int realAngle180;//Opposite of the front of the moviment

	if (realRobotAngle < 180)
	{
		realAngle180 = realRobotAngle + 180;
		if ((compass.value() >= realRobotAngle) && (compass.value() < realAngle180))
			error = compass.value() - realRobotAngle;
		else
		{
			if ((compass.value() >= 0) && (compass.value() < realRobotAngle))
			{
				error = realRobotAngle - compass.value();
			}
			else
			{
				error = realRobotAngle + (360 - compass.value());
			}
		}
	}
	else
	{
		realAngle180 = realRobotAngle - 180;
		if ((compass.value() <= realRobotAngle) && (compass.value() > realAngle180))
		{
			error = realRobotAngle - compass.value();
		}
		else
		{
			if (compass.value() < realAngle180)
				error = (360 - realRobotAngle) + compass.value();
			else
			{
				error = compass.value() - realRobotAngle;
			}
		}
	}
	//error Ok

	if (error < 0.5)
		error = 0;
	else if (error < 1)
		error = 1;
	if (error > 89)
		error = 89;

	//cout << "PID(error):" << error << endl;									//able to see the PID(error) value

	if (error < 2)
	{
		return (1 / (1 + error*Kp*0.6));//Max error= 180 degree
	}
	else if (error < 6)
	{
		return (1 / (1 + error*Kp*1.5));//Max error= 180 degree
	}
	else if (error < 90)
	{
		return (1 / (1 + error*Kp*2));//Max error= 180 degree
	}
}

void Motors::simpleMove(int moveAngle, int frontRobotAngle)
{
	if (moveAngle != -1)//--------------------------------------------------------(normal)
	{
		compass.update();
		while (moveAngle >= 360)//more than 360 angle tratament
			moveAngle -= 360;
		while (moveAngle < 0)//negative angle tratament
			moveAngle += 360;

		while (frontRobotAngle >= 360)//more than 360 angle tratament
			frontRobotAngle = frontRobotAngle - 360;
		while (frontRobotAngle < 0)//negative angle tratament
			frontRobotAngle = frontRobotAngle + 360;

		bool distance90 = isNear(moveAngle, frontRobotAngle, 90);//true=is near

		float compass180 = compass.value();// 0 -> 45 -> 90 -> 135 -> 180 -> -135 -> -90 -> -45-> 0

		if (compass180 > 180)
		{
			compass180 = (360 - compass180)*(-1);
		}

		float robotAngle180 = moveAngle;
		if (robotAngle180 > 180)
		{
			robotAngle180 = (360 - robotAngle180)*(-1);
		}

		float alpha = compass180;
		float beta = robotAngle180;

		//cout << "alpha: " << alpha<<endl;
		//cout << "beta: " << beta<<endl;
		//-----------------------------------------spin calcule
		int spinTimes = 0;//Number of spins

		if (compass.value() > 180)
			spinTimes = 4;

		int spinTime_auxiliary = 0;
		int RealNumberMotor;//The real number after degree tratament

		if (compass.value() <= 180)
		{
			while (compass.value() - 45 - (90 * spinTime_auxiliary) >= 0)
			{
				spinTimes++;
				spinTime_auxiliary++;
			}
		}
		else
		{
			while ((360 - compass.value()) - 45 - (90 * spinTime_auxiliary) >= 0)
			{
				spinTimes--;
				spinTime_auxiliary++;
			}
		}

		if (spinTimes < 0)
			spinTimes = 4 + spinTimes;
		//-----------------------------------------spin calcule

		float angle = 45 + alpha + beta - spinTimes * 90;//the robot always will move in the robotAngle angle
		while (angle < 0)
			angle = angle + 360;
		while (angle >= 360)
			angle = angle - 360;

		angle = angle*M_PI / 180;//transform moviment angle trigonometric circle(radians)

		double diagonalLeft = sin(angle);
		double diagonalRight = cos(angle);

		if (sin(angle) < 0)//value tratament: positive
			diagonalLeft = diagonalLeft*(-1);
		if (cos(angle) < 0)
			diagonalRight = diagonalRight*(-1);

		float correction = 1;
		//value tratament: if diagonal>1, diagonal=1 (the other decreases proportion)
		if (diagonalRight > diagonalLeft)
			correction = 1 / diagonalRight;
		else
			correction = 1 / diagonalLeft;

		if (sin(angle) < 0)//value tratament: positive
			diagonalLeft = diagonalLeft*(-1);
		if (cos(angle) < 0)
			diagonalRight = diagonalRight*(-1);

		

		diagonalRight = diagonalRight*correction * 255 * (static_cast<float>(maxPower) / 255);//float power
		diagonalLeft = diagonalLeft*correction * 255 * (static_cast<float>(maxPower) / 255);//float power

		float PID = PIDController(360 - frontRobotAngle);

		float compassRobot = (360 - compass.value()) - frontRobotAngle;//the 0 is the frontRobotAngle when the robot move

		while (compassRobot < 0)
			compassRobot += 360;
		while (compassRobot >= 360)
			compassRobot -= 360;

		if (compassRobot < 180)//Fast motion correction
		{
			if (diagonalLeft >= 0)
			{
				relativeMotor(FR, diagonalLeft*PID);
				relativeMotor(BL, diagonalLeft*(1 / PID));
			}
			else
			{
				relativeMotor(FR, diagonalLeft*(1 / PID));
				relativeMotor(BL, diagonalLeft*PID);
			}
			if (diagonalRight >= 0)
			{
				relativeMotor(FL, diagonalRight*(1 / PID));
				relativeMotor(BR, diagonalRight*PID);
			}
			else
			{
				relativeMotor(FL, diagonalRight*PID);
				relativeMotor(BR, diagonalRight*(1 / PID));
			}
		}
		else if (compassRobot >= 180)//Fast motion correction 
		{
			if (diagonalLeft >= 0)
			{
				relativeMotor(FR, diagonalLeft*(1 / PID));
				relativeMotor(BL, diagonalLeft*PID);
			}
			else
			{
				relativeMotor(FR, diagonalLeft*PID);
				relativeMotor(BL, diagonalLeft*(1 / PID));
			}
			if (diagonalRight >= 0)
			{
				relativeMotor(FL, diagonalRight*PID);
				relativeMotor(BR, diagonalRight*(1 / PID));
			}
			else
			{
				relativeMotor(FL, diagonalRight*(1 / PID));
				relativeMotor(BR, diagonalRight*PID);
			}
		}
	}
	else//-------------------------------------------------------------------(spin)
	{
		float PID = PIDController(360 - frontRobotAngle);

		float compassRobot = (360 - compass.value()) - frontRobotAngle;//the 0 is the frontRobotAngle when the robot move

		while (compassRobot < 0)
			compassRobot += 360;
		while (compassRobot >= 360)
			compassRobot -= 360;

		if (compassRobot <= 5)//Stop
		{
			relativeMotor(FR, 0);
			relativeMotor(BL,0);

			relativeMotor(FL, 0);
			relativeMotor(BR, 0);
		}
		else if (compassRobot <= 20)//Slow motion correction 
		{
			relativeMotor(FR, maxPower*PID*(-1));
			relativeMotor(BL, maxPower*PID);

			relativeMotor(FL, maxPower*PID);
			relativeMotor(BR, maxPower*PID*(-1));
		}
		else if (compassRobot < 180)//Fast motion correction
		{

			relativeMotor(FR, maxPower*(1 / PID)*(-1));
			relativeMotor(BL, maxPower*(1 / PID));

			relativeMotor(FL, maxPower*(1 / PID));
			relativeMotor(BR, maxPower*(1 / PID)*(-1));

		}
		if (compassRobot >= 355)//Stop
		{
			relativeMotor(FR, 0);
			relativeMotor(BL, 0);

			relativeMotor(FL, 0);
			relativeMotor(BR, 0);
		}
		else if (compassRobot >= 340)//Slow motion correction 
		{
			relativeMotor(FR, maxPower*PID);
			relativeMotor(BL, maxPower*PID*(-1));

			relativeMotor(FL, maxPower*PID*(-1));
			relativeMotor(BR, maxPower*PID);

		}
		else if (compassRobot >= 180)//Fast motion correction 
		{

			relativeMotor(FR, maxPower*(1/PID));
			relativeMotor(BL, maxPower*(1 / PID)*(-1));

			relativeMotor(FL, maxPower*(1 / PID)*(-1));
			relativeMotor(BR, maxPower*(1 / PID));

		}
	}
}

bool Motors::isNear(float firstNumber, float secondNumber, float discrepancy)//return false if discrepancy > distance of the numbers
{
	bool isNear = true;

	if (firstNumber - secondNumber > discrepancy)
		isNear = false;
	else if (secondNumber - firstNumber > discrepancy)
		isNear = false;

	return isNear;
}