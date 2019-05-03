// Library (Camera) robô de futebol 2018 CIC Robotics
// 17/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#if !defined(ARDUINO)
extern "C" {
#include "extApi.h"
}
#endif

class Compass;
class DataBase;

#ifndef CAMERA_H
#define CAMERA_H

class Camera
{
public:
	Camera(DataBase &,Compass &);

	void uptadeXYBall();
	void setServoAngle(int angle);
	void updateServoAngle();

	bool getIBallKnowed();
	int getBallX();
	int getBallY();

	//Simulation
	void initializeSimulation(int ID);//initialize the simulation camera and ID

private:
	void readCamera();
	
	float servoAngle;

	int contObjects;//number of "balls"(more than one is strange)
	float imageX;//X coordinate in the image
	float imageY;//Y coordinate in the image
	float imageHeight;//height in the image
	float ballDistance;
	//-----objects-----//
	Compass &compass;
	DataBase &dataBase;
	//-----const values-----//
	const double Kd;//used in distance calculation
	const float ballDiameter;
	const float FieldOfViewHorizontal;
	const float FieldOfViewVertical;
	const float robotCamHeight;

#if !defined(ARDUINO)//Simulation
	int clientIDSimulation;//ID
	int servoSimulation;//handle
	int cameraSimulation;//handle

	float* auxCameraValues;//cam auxiliary
	int* auxCameraValuesCount;//cam auxiliary

	simxUChar state;
#endif
};

#endif // CAMERA_H