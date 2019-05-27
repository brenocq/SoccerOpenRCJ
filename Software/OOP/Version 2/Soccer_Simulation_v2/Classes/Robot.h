///////////////////////////////////////////////////////////
//  Robot.h
//  Implementation of the Class Robot
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef ROBOT_H
#define ROBOT_H

#include "Ultrasonic.h"
#include "Compass.h"
#include "LightSensor.h"
#include "Camera.h"
#include "Motor.h"
#include "Servo.h"
#include "Solenoid.h"
#include "MotorsEnum.h"
#include "FieldMatrix.h"
#include "TimeThread.h"
#include "Strategy.h"
#include "Display.h"
#include "ExternalCommunication.h"
#include <vector>
#include <cassert>//assert
using namespace std;

class ExternalCommunication;

class Robot
{

public://------------------------------------------------------//public//
#if defined(ARDUINO)
	Robot();
#else
	enum RobotMode { ATTACKER, DEFENDER };

	Robot(int number, int clientID, ExternalCommunication *_bluetooth, RobotMode _mode);
#endif
	virtual ~Robot();

	void execute();

	//----- GAME -----//
	bool blockGoal();
	bool goToDefense();

	bool kickToGoal();
	bool catchTheBall();
	float goalProbability();
	bool ballPass();
	//----- Moviment -----//
	float frontPID(float frontAngle);
	bool simpleMove(float frontRobotAngle = 1000, float moveAngle = 1000);//simple moviment with move angle and front angle
	bool goToPosition(int X, int Y, float frontRobotAngle, bool smooth=false);// simple go to position, don't deviate (return true if arrived on position)
	
	bool vectorMove(vector<int>& path, float frontRobotAngle);// create the best path to arrive the place
	void circleAvoidObstacle(vector<int>& path, int Xcircle, int Ycircle, int goalX, int goalY, int radius, int oppositeAngle, int sense = 0, int recursion = 20);//create path to deviate obstacles
	//----- Path -----//
	bool generateBallPath(vector<int>& path);// create the best path to arrive the place
	bool generatePathLine(vector<int>& path, int x0, int y0, int x1, int y1, int scale);// create the best path to arrive the place
	bool generatePathArcThreePoints(vector<int>& pathPoints, float x1, float y1, float x2, float y2, float x3, float y3, int scale);

	//----- Motor -----//
	void relativeMotor(MotorsEnum motorName, int motorPower);
	//----- Solenoid -----//
	void kick();
	//----- Servo -----//
	void updateServo();
	//----- Ultrasonic -----//
	Ultrasonic relativeSonar(int sonarNumber);//return the object sonar
	void updateSonar();
	void updateSonarValuesAngles();
	void updateSonarPointX();
	void updateSonarPointY();
	//----- Light Sensor -----//
	void updateLightSensor();
	//----- Compass -----//
	void updateCompass();
	//----- Camera -----//
	void updateCamera();
	//----- Bluetooth -----//
	void readBluetooth();
	void writeBluetooth();
	char mergeBoolBluetooth(bool b1, bool b2, bool b3, bool b4, bool b5, bool b6, bool b7, bool b8);
	//----- Robot -----//
	void updateRobotX();
	void updateRobotY();
	void updateRobotAngle();
	//----- Adversary -----//
	void updateAdversary();
	//----- Simulation -----//
	void initializeSimulation(int ID);
	//----- Auxiliary functions -----//
	bool isNear(float firstNumber, float secondNumber, float discrepancy);
	bool isNearAngle(float firstNumber, float secondNumber, float discrepancy);
	float distanceTwoAngles(float firstNumber, float secondNumber);
	bool isAngleTimeClock(float firstNumber, float secondNumber);
	float angleTwoPoints(int x1, int y1, int x2, int y2);
	float distanceTwoPoints(int x1, int y1, int x2, int y2);
	void risingVector(int vector[], int length);

	//----- Field monitoring -----//
	void delAlonePoints();//delete alone points in the field
	void drawSonarPoints();//draw detected points by the sonars
	void sonarCleaning();//delete points detected between the robot and sonar point

	//----- train robot -----//
#if !defined(ARDUINO)
	bool randomMoviment(vector<float> &realXY);
#endif
	//-----Get property-----//
	//Motor
	int getMaxPower() const					{ return maxPower; }
	//Servo
	int GetServoAngle()const				 { return servoAngle; }
	//Ultrasonic
	float GetSonarValues(int sonarNum) const	{ return sonarValues[sonarNum]; }
	//Compass
	double GetCompassValue()const			{ return compassValue; }
	//Robot
	int GetRobotRatio()const				{ return robotRatio; }
	bool GetIsInTheGame()const				{ return isInTheGame; }
	int GetRobotNumber()const				{ return robotNumber; }
	//RobotX
	int GetRobotX()const					{ return robotX; }
	bool GetIsFirstRobotX()const			{ return isFirstRobotX; }
	bool GetIsRobotXKnowed()const			{ return isRobotXKnowed; }
	int GetDiscrepancyX()const				{ return discrepancyX; }
	//RobotY
	int GetRobotY()const					{ return robotY; }
	bool GetIsFirstRobotY()const			{ return isFirstRobotY; }
	bool GetIsRobotYKnowed()const			{ return isRobotYKnowed; }
	int GetDiscrepancyY()const				{ return discrepancyY; }
	//Adversary
	int* GetAdversaryXY() { return adversaryXY; }
	int GetNumAdversaries() { return numAdversaries; }
	//Friend
	int* GetFriendXY()						{ return friendXY; }
	bool GetIsFriendKnowed()				{ return isFriendKnowed; }
	//Ball
	float GetBallDiameter()const			{ return ballDiameter; }
	int GetBallX()const						{ return ballX; }
	int GetBallY()const						{ return ballY; }
	bool GetIsBallKnowed()const				{ return isBallKnowed; }
	bool GetRobotWithBall()const			{ return robotWithBall; }
	int GetSRobotHandle()const				{ return SimulationRobotHandle; }

	//-----Set property-----//
	void setMaxPower(int newVal)			 { maxPower = newVal; }
	void SetRobotX(int newVal)				 { robotX = newVal; }
	void SetRobotY(int newVal)				 { robotY = newVal; }
	void SetIsInTheGame(bool newVal)		 { isInTheGame = newVal; }
	void SetIsFirstRobotX(bool newVal)		 { isFirstRobotX = newVal; }
	void SetIsRobotXKnowed(bool newVal)		 { isRobotXKnowed = newVal; }
	void SetDiscrepancyX(int newVal)		 { discrepancyX = newVal; }
	void SetIsFirstRobotY(bool newVal)		 { isFirstRobotY = newVal; }
	void SetIsRobotYKnowed(bool newVal)		 { isRobotYKnowed = newVal; }
	void SetDiscrepancyY(int newVal)		 { discrepancyY = newVal; }
	void SetBallX(int newVal)				 { ballX = newVal; }
	void SetBallY(int newVal)				 { ballY = newVal; }
	void SetIsBallKnowed(bool newVal)		 { isBallKnowed = newVal; }
	void SetRobotWithBall(bool newVal)		 { robotWithBall = newVal; }
	void SetFriendXY(int valX, int valY)	 { friendXY[0] = valX; friendXY[1] = valY; }
	bool SetIsFriendKnowed(bool newVal)		 { isFriendKnowed = newVal; }
	void SetCompassValue(double newVal)		 { compassValue = newVal; }
	void SetRobotNumber(int newVal)			 { robotNumber = newVal; }
	void SetServoAngle(int newVal)			 { servoAngle = newVal; }

	//----- Consulted by gameControl -----//
	int sonarPointX[12];
	int sonarPointY[12];
	FieldMatrix *fieldMatrix;
	vector<int>pathRobot;
	//only for simulation
#if !defined(ARDUINO)
	float kicked;
#endif
private://----------------------------------------------------//private//
	//----- Robot -----//

	int robotMode;
	float frontAngle;
	//----- Team -----//
	bool teamWithBall;

	//Object-Sensors
	Ultrasonic *ultrasonic1; Ultrasonic *ultrasonic2; Ultrasonic *ultrasonic3; Ultrasonic *ultrasonic4;
	Ultrasonic *ultrasonic5; Ultrasonic *ultrasonic6; Ultrasonic *ultrasonic7; Ultrasonic *ultrasonic8;
	Ultrasonic *ultrasonic9; Ultrasonic *ultrasonic10; Ultrasonic *ultrasonic11; Ultrasonic *ultrasonic12;
	Compass *compass;
	LightSensor *lightBallPos;
	Camera *camera;
	//Object-Actuators
	Motor *motorFR; Motor *motorBR; Motor *motorBL; Motor *motorFL; Motor *motorXicoR;
	Servo *servo;
	Solenoid *solenoid;
	//Object-Display
	Display *display;
	//Object-Strategy
	Strategy *strategy;
	//Object-Bluetooth
	ExternalCommunication *bluetooth;
	//Object-Thread
	TimeThread *timeThreadUpdate1;
	TimeThread *increaseDiscrepancyX;
	TimeThread *increaseDiscrepancyY;
	TimeThread *frontAngleRate;
	TimeThread *debug1;
	TimeThread *movement;

	//Strategy
	int myPassState;
	int friendPassState;
	//Robot
	const int robotRatio;
	int robotX;
	int robotY;
	bool isInTheGame;
		//robotX
		bool isFirstRobotX;
		bool isRobotXKnowed;
		int discrepancyX;	
		//robotY
		bool isFirstRobotY;
		bool isRobotYKnowed;
		int discrepancyY;
	//Ball
	const float ballDiameter;
	int ballX;
	int ballY;
	float ballXcam;
	float ballYcam;
	bool myIsBallKnowed;
	bool isBallKnowed;
	bool isBallRight;
	bool robotWithBall;
	//Goal
	bool isGoalKnowed;
	const int realGoalHeight;
	float realGoalLength;//need to be calculated
	float goalXCam;
	float goalYCam;
	float goalLengthCam;
	float goalHeightCam;
	//Adversary
	int adversaryXY[25];
	int numAdversaries;
	//Friend
	int friendXY[2];
	bool isFriendKnowed;
	//Motor
	vector <Motor*> motors;
	int maxPower;
		//PID control
	double Kp,Ki,Kd;//PID constants
	double P, I, D;
	unsigned long lastPIDprocess;
	double lastDegree;
	bool isFirstPID;
	//Servo
	int servoAngle;
	//Compass
	double compassValue;
	//Ultrasonic
	vector <Ultrasonic*> ultrasonics;
	float sonarValues[12];
	float sonarAngles[12];
	int spinTimesSonar;
	//LightSensor
	int LSFieldValue[9];
	int LSBallValue;
	//Simulation
	int clientIDSimulation;
	int robotNumber;
	int SimulationRobotHandle;
	int SimulationDribbleHandle;
	//Simulation Ball possession control
	int SimulationBallHandle;
	int SimulationSuctionPointHandle;
	int SimulationSuctionPadHandle;

};
#endif // ROBOT_H
