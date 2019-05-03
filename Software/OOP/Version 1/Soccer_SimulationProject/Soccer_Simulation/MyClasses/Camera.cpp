// Library (FieldMatrix) robô de futebol 2018 CIC Robotics
// 17/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "Camera.h"
#include "DataBase.h"
#include "Compass.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else

#include <cstdint>//int16_t values
#include <string>//string
#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;
#include <iomanip>//setprecision
using std::setprecision;
using std::setprecision;

extern "C" {
#include "extApi.h"
}
#endif

Camera::Camera(DataBase &robotDataBase, Compass &robotCompass)
	:compass(robotCompass),
	dataBase(robotDataBase),
	clientIDSimulation(-1),
	servoSimulation(-1),
	cameraSimulation(-1),
	servoAngle(60),//servo starts with 60degrees
	ballDiameter(6.5),
	Kd(0.207*20/6.5),//(height*distance/sizeBall)
	FieldOfViewHorizontal(75),//pixy-75degrees
	FieldOfViewVertical(75),//pixy-47degrees
	robotCamHeight(18)//18cm above ground
{
	setServoAngle(servoAngle);
}

void Camera::initializeSimulation(int ID)
{
#if !defined(ARDUINO)
	clientIDSimulation = ID;

	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*) "Omni_Servo", (simxInt *)&servoSimulation, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << "Pixy servo handle not found!" << std::endl;
	else
		cout << "Connected to the pixy servo!" << std::endl;

	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*) "PixyCam", (simxInt *)&cameraSimulation, (simxInt)simx_opmode_blocking) != simx_return_ok)
		cout << "PixyCam handle not found!" << std::endl;
	else
		cout << "Connected to the pixyCam!" << std::endl;
	
	simxReadVisionSensor(clientIDSimulation, cameraSimulation, &state, &auxCameraValues, &auxCameraValuesCount, simx_opmode_streaming);
#endif
}

void Camera::setServoAngle(int angle)
{
	servoAngle = angle;
	simxSetJointPosition(clientIDSimulation, servoSimulation, static_cast<float>(angle)*M_PI / 180, simx_opmode_streaming);
}

void Camera::readCamera()
{
	if (simxReadVisionSensor(clientIDSimulation, cameraSimulation, &state, &auxCameraValues, &auxCameraValuesCount, simx_opmode_buffer) == simx_return_ok)
	{
		contObjects = auxCameraValues[15];
		if (contObjects && (auxCameraValues[21] / auxCameraValues[22]>0.9))
		{
			dataBase.setIsBallKnowed(true);
			if (auxCameraValues[21] > auxCameraValues[22])
				imageHeight = auxCameraValues[21];
			else
				imageHeight = auxCameraValues[22];

			imageX = auxCameraValues[19];
			imageY = auxCameraValues[20];

			ballDistance = (ballDiameter*Kd / imageHeight);

			double errorX = abs(0.5 - imageX);
			double errorY = abs(0.5 - imageY);

			
			if ((errorX < 0.15 && errorY < 0.15) && ballDistance<85)
			{
				float totalCamHeight = robotCamHeight;
				float XTotalDistance = tan(servoAngle*M_PI / 180)*totalCamHeight;
				float XExceededDistance = tan(servoAngle*M_PI / 180)*ballDiameter/2;

				ballDistance = XTotalDistance - XExceededDistance;
			}
		}
		else
		{
			dataBase.setIsBallKnowed(false);
		}
			/*cout<< setprecision(3);
			cout << " X:" << imageX << " Y:" << imageY << " H:" << imageHeight << " D:" << ballDistance;
			cout<< endl;*/
	}
}

void Camera::updateServoAngle()
{
	readCamera();
	if (dataBase.getIsBallKnowed())
	{
		float errorY = (0.5 - imageY)*(-1);
		if (abs(errorY) > 0.05)
		{
			servoAngle = servoAngle + (FieldOfViewVertical / 2)*errorY;
		}
		setServoAngle(servoAngle);
	}
	else
	{
		setServoAngle(60);
	}
}

void Camera::uptadeXYBall()
{
	updateServoAngle();//readCamera and update the servo angle
	readCamera();//readCamera again

	if (dataBase.getIsRobotXKnowed() && dataBase.getIsRobotYKnowed() && dataBase.getIsBallKnowed())
	{
		float ballAngle = -compass.value() + FieldOfViewHorizontal*(0.5-imageX);
		int relativeBallX = ballDistance*sin(ballAngle*M_PI / 180);
		int relativeBallY = ballDistance*cos(ballAngle*M_PI / 180);
		dataBase.setBallX(dataBase.getRobotX() - relativeBallX);
		dataBase.setBallY(dataBase.getRobotY() + relativeBallY + 6);

		if (imageX >= 0.5)
			dataBase.setLastBallWasRight(true);
		else
			dataBase.setLastBallWasRight(false);

		dataBase.setRobotBallAngle(ballAngle);
	}
	else
	dataBase.setIsBallKnowed(false);
}