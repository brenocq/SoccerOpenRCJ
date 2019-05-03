// Library (Ultrasonic) robô de futebol 2018 CIC Robotics
// 16/01/2018
// Desenvolvedor: Breno Cunha Queiroz

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "TimeThread.h"

class Compass;
class DataBase;

class Ultrasonic
{
public:
	Ultrasonic(DataBase &,Compass &);

	void initializeSimulation(int ID);//initialize the simulation ultrasonics
	float value(int sonarNumber);
	float angle(int numberSonar, bool isRecursion = false);
	void updatePointX();
	void updatePointY();
	int getPointX(int sonarNumber);
	int getPointY(int sonarNumber);
	bool getIsXknowed();
	bool getIsYknowed();
	int getRobotX();
	int getRobotY();

	void updateObstacleReading();//put this in another class
	void updateAdversaryXY();//put this in another class

private:
	bool isNear(float firstNumber, float secondNumber, float discrepancy);
	void risingVector(int vector[], int length);
	int sonarPointX[12];
	int sonarPointY[12];
	int possibleRobotX;
	int possibleRobotY;
	bool isFirstRobotX;
	bool isFirstRobotY;
	int widthField;
	int lengthField;
	int discrepancyX;
	int discrepancyY;
	//Simulation
	int sonarHandle[12];//ultrasonics handles
	int clientIDSimulation;
	//Objects
	Compass &compass;
	DataBase &dataBase;
	TimeThread *increaseDiscrepancyX;
	TimeThread *increaseDiscrepancyY;
};

#endif // ULTRASONIC_H