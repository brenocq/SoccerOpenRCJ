// Simulação do robô de futebol 2018 CIC Robotics
// 12/12/2017
// Desenvolvedor: Breno Cunha Queiroz


//---------------------------------------------------------------INCLUDE-------------------------------------------------------------

#define _USE_MATH_DEFINES

#include <iostream>
using namespace std;
#include <stdio.h>
#include <string>
#include <cmath>//Pi,cos,sin,atan
#include "stdafx.h"
#include <cstdlib>//to clear the screen

#include <algorithm>//to swap function
#include <windows.h>//to see pixels
#include <time.h>//to thread

#include "MyClasses/Motors.h"
#include "MyClasses/Compass.h"
#include "MyClasses/Ultrasonic.h"
#include "MyClasses/Camera.h"
#include "MyClasses/FieldMatrix.h"
#include "MyClasses/FieldDraw.h"
#include "MyClasses/TimeThread.h"
#include "MyClasses/Dribble.h"
#include "MyClasses/BPSensor.h"
#include "MyClasses/Solenoid.h"

#include "MyClasses/BitMatrix.h"
#include "MyClasses/DataBase.h"

extern "C" {
#include "remoteAPI/extApi.h"
}

//-----------------------------------------------------------GLOBAL VARIABLES---------------------------------------------------------
//-----simulation-----//
int ID = 0;

//-----See Field-----//
HWND myconsole = GetConsoleWindow();
HDC mydc = GetDC(myconsole);
bool fieldWasDrawn = false;
//-----Ball-----//
bool isBallKnowed = true;
int ballX = 45;
int ballY = 182;
//-----Generate Route-----//
int pathPoints[20] = { 0 };
int bestPath[20] = { 0 };

TimeThread erasePoints(3000);
TimeThread debug(1000);
TimeThread debug1(500);
TimeThread testVelocity(0);
//-----------------------------------------------------------START FUNCTIONS----------------------------------------------------------

int Initial()
{
	simxChar *DommTest;
	DommTest = "Dummy";
	simxInt *check = new simxInt[1];
	check[0] = 0;
	check[1] = 0;
	int portNb = 19998;
	int clientID = -1;
	simxFinish(-1);

	clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);

	if (clientID > -1)
	{

		simxGetObjectHandle(clientID, DommTest, check, simx_opmode_oneshot_wait);
		printf("Connection Success ... \n");
		printf("\n Number of ID:  %d", clientID);
		printf("\n check: %d", check);
		// extApi_sleepMs(10);
	}
	else
	{
		printf("Connection Fail");
		getchar();
		extApi_sleepMs(10);
	}

	return clientID;
}

int fStart_Stop(int ID)
{
	printf("\nPlease Enter Y/N for Start Stop Simulation ");
	char Star_Stop = getchar();
	printf("\n");
	int start = -1;
	if (Star_Stop == 'Y')
	{
		printf("Starting ...\n");
		start = simxStartSimulation(ID, simx_opmode_oneshot);
	}
	if (Star_Stop == 'N')
	{
		printf("Closing...\n");
		start = simxStopSimulation(ID, simx_opmode_oneshot);
		//for(int i = 0 ; i< 100000;i++) i++;
	}
	/*else
	printf("Closing...");
	start = simxStopSimulation(ID,simx_opmode_oneshot);*/
	return start;
}

//----------------------------------------------------------AUXILIARY FUNCTIONS-------------------------------------------------------

/*bool isNear(float firstNumber, float secondNumber, float discrepancy)//return false if discrepancy > distance of the numbers
{
	bool isNear = true;

	if (firstNumber - secondNumber > discrepancy)
		isNear = false;
	else if (secondNumber - firstNumber > discrepancy)
		isNear = false;

	//cout << "Near(1N:" << firstNumber << ", 2N:" << secondNumber << ", maxDistance:" << discrepancy << ")= " << isNear << "\n";
	return isNear;

}

void risingVector(int vector[], int length)
{
	int temp;

	for (int i = 0; i < length; i++)
	{
		for (int j = i + 1; j < length; j++)
		{
			if (vector[i]>vector[j])
			{
				temp = vector[i];
				vector[i] = vector[j];
				vector[j] = temp;
			}
		}
	}
}

//--------------------------------------------------------------FUNCTIONS-------------------------------------------------------------
void updateGlobalFrontRobotAngle(int clientID)
{
	if (isBallKnowed == true)
	{
		globalFrontRobotAngle = angleTwoPoints(robotX, robotY, ballX, ballY);
	}
}

void updateGoalAngle(int clientID)
{
	isInPosition = false;
	goalAngle = angleTwoPoints(robotX, robotY, goalX, goalY);
}

void goToPosition(int clientID, int positionX, int positionY, int frontRobotAngle, float maxPower)
{
	int Xdistance = robotX - positionX;
	int Ydistance = robotY - positionY;
	int movementAngle = 0;
	if (Xdistance < 0)
		Xdistance *= (-1);
	if (Ydistance < 0)
		Ydistance *= (-1);

	if ((Xdistance > 5 && Ydistance > 5) || (Xdistance > 5 || Ydistance > 5))
	{
		isInPosition = false;

		movementAngle = angleTwoPoints(robotX, robotY, positionX, positionY);
		/*if (positionY > robotY)//   ball /\  /
		{
		if (positionX > robotX)//   ball>
		{
		movementAngle = (-1)*int(atan2(Xdistance, Ydistance) * 180 / M_PI);
		}
		else if (positionX < robotX)//   ball<
		{
		movementAngle = int(atan2(Xdistance, Ydistance) * 180 / M_PI);
		}
		else//   ball |
		{
		movementAngle = 0;
		}

		}
		else if (positionY < robotY)//   ball\/
		{
		if (positionX > robotX)//   ball >
		{
		movementAngle = (-1)*int(90 + (90 - (atan2(Xdistance, Ydistance) * 180 / M_PI)));
		}
		else if (positionX < robotX)//   ball <
		{
		movementAngle = (-1)*int(180 + atan2(Xdistance, Ydistance) * 180 / M_PI);
		}
		else//   ball|
		{
		movementAngle = 180;
		}
		}
		else
		{
		if (positionX > robotX)
		{
		movementAngle = 90;
		}
		else if (positionX < robotX)
		{
		movementAngle = -90;
		}
		}

		move(ID, movementAngle, frontRobotAngle, maxPower);
	}
	else
	{
		isInPosition = true;
		//cout << "I reached the goal" << endl;
		move(ID, movementAngle, frontRobotAngle, 0);
	}
}
//-----avoid obstacles-----//
void CircleAvoidObstacle(int Xcircle, int Ycircle, int radius, int oppositeAngle, int sense = 0, int recursion = 10)
{
	int firstOppositeAngle = oppositeAngle;
	//sense=0	clockwise and anti-clockwise
	//sense=1	clockwise
	//sense=2	anti-clockwise
	bool clockwise = true;
	int pointsFounded = 2;

	int lastPointPath = -1;
	for (int i = 0; pathPoints[i] > 0; i++)
	{
		lastPointPath = i;
	}
	pathPoints[lastPointPath + 1] = Xcircle;
	pathPoints[lastPointPath + 2] = Ycircle;

	int circleX1 = 0;
	int circleY1 = 0;
	int circleX2 = 0;
	int circleY2 = 0;

	if (sense == 2)
		clockwise = false;
	if (sense != 0)
		pointsFounded--;

	int contWhile = 0;

	while ((clockwise == true && pointsFounded > 0) && (contWhile < 80))
	{
		float trigonometricAngle = (oppositeAngle + 90)*M_PI / 180;//angle in radians
		circleX2 = Xcircle + int(radius*cos(trigonometricAngle));
		circleY2 = Ycircle + int(radius*sin(trigonometricAngle));
		if (matrixFillCircleRead(circleX2, circleY2, radius, 1) == 0)
		{
			//drawCircle(circleX2, circleY2, radius, 5);
			pointsFounded--;
			if (pointsFounded == 1 && sense == 0)
			{
				clockwise = false;
				oppositeAngle = firstOppositeAngle;
			}
		}
		drawFillCircle(circleX2, circleY2, 3, 2);
		oppositeAngle -= 5;
		contWhile++;
	}
	contWhile = 0;
	while ((clockwise == false && pointsFounded > 0) && (contWhile < 80))
	{
		float trigonometricAngle = (oppositeAngle + 90)*M_PI / 180;//angle in radians
		circleX1 = Xcircle + int(radius*cos(trigonometricAngle));
		circleY1 = Ycircle + int(radius*sin(trigonometricAngle));
		if (matrixFillCircleRead(circleX1, circleY1, radius, 1) == 0)
		{
			//drawCircle(circleX1, circleY1, radius, 5);
			pointsFounded--;
		}
		drawFillCircle(circleX1, circleY1, 3, 5);
		oppositeAngle += 5;
		contWhile++;
	}
	//-----see circles-----//
	if (distTwoPoints(circleX1, circleY1, goalX, goalY) < distTwoPoints(circleX2, circleY2, goalX, goalY))
	{
		drawCircle(circleX1, circleY1, radius, 2);
	}
	else
	{
		drawCircle(circleX2, circleY2, radius, 2);
	}

	//----------//
	if (recursion == 10)
	{
		if ((distTwoPoints(circleX1, circleY1, goalX, goalY) <= distTwoPoints(circleX2, circleY2, goalX, goalY))
			&& matrixBigLineRead(Xcircle, Ycircle, goalX, goalY, 11, 1, 1) != 0)
		{
			//cout << "clock";
			oppositeAngle = angleTwoPoints(circleX1, circleY1, goalX, goalY);
			drawCircle(circleX1, circleY1, radius, 2);
			CircleAvoidObstacle(circleX1, circleY1, radius, oppositeAngle, 2, recursion - 1);

		}
		else if ((distTwoPoints(circleX2, circleY2, goalX, goalY) <= distTwoPoints(circleX1, circleY1, goalX, goalY)) &&
			(matrixBigLineRead(Xcircle, Ycircle, goalX, goalY, 11, 1, 1) != 0))
		{
			//cout << "antclock";
			oppositeAngle = angleTwoPoints(circleX2, circleY2, goalX, goalY);
			drawCircle(circleX2, circleY2, radius, 2);
			CircleAvoidObstacle(circleX2, circleY2, radius, oppositeAngle, 1, recursion - 1);

		}
	}
	else if (recursion > 0)
	{
		//cout << "bat";
		if ((matrixBigLineRead(Xcircle, Ycircle, goalX, goalY, 11, 1, 1) != 0) && sense == 2)
		{
			oppositeAngle = angleTwoPoints(circleX1, circleY1, goalX, goalY);
			drawCircle(circleX1, circleY1, radius, 2);
			CircleAvoidObstacle(circleX1, circleY1, radius, oppositeAngle, 2, recursion - 1);
		}
		else if ((matrixBigLineRead(Xcircle, Ycircle, goalX, goalY, 11, 1, 1) != 0) && sense == 1)
		{
			oppositeAngle = angleTwoPoints(circleX2, circleY2, goalX, goalY);
			drawCircle(circleX2, circleY2, radius, 2);
			CircleAvoidObstacle(circleX2, circleY2, radius, oppositeAngle, 1, recursion - 1);
		}
	}
}
void generateRoute(int clientID)//uses goalX and goalY
{
	/*
	1: Algorithm to read the area that the robot would pass to reach the objective,
	Create four points (all tangential to the robot line -> objective)
	Make lines that start from the robot to the target in the set radius
	*//*
	int numberLines = 1;

	int lineRatio = 17;
	int lineDistance = 1;

	for (int i = 0; pathPoints[i] > 0; i++)
		pathPoints[i] = 0;

	drawLine(robotX, robotY, goalX, goalY, 4);
	//-----temp variables-----//
	float trigonometricAngle = 0;
	//-----left point robot-----//
	int pointLeftAngleRobot = goalAngle + 90;//tangencial angle

	int pointXLeftRobot = 0;
	int pointYLeftRobot = 0;

	trigonometricAngle = (pointLeftAngleRobot + 90)*M_PI / 180;//angle in radians
	pointXLeftRobot = robotX + int(lineRatio*cos(trigonometricAngle));
	pointYLeftRobot = robotY + int(lineRatio*sin(trigonometricAngle));

	//matrixFillCircleWrite(pointXLeftRobot, lengthField - pointYLeftRobot, 3, 4);
	//-----right point robot-----//
	int pointRightAngleRobot = goalAngle - 90;//tangencial angle

	int pointXRightRobot = 0;
	int pointYRightRobot = 0;

	trigonometricAngle = (pointRightAngleRobot + 90)*M_PI / 180;//angle in radians
	pointXRightRobot = robotX + int(lineRatio*cos(trigonometricAngle));
	pointYRightRobot = robotY + int(lineRatio*sin(trigonometricAngle));

	//matrixFillCircleWrite(pointXRightRobot, lengthField - pointYRightRobot, 3, 4);
	//-----point point goal-----//
	int pointLeftAngleGoal = goalAngle - 90;//tangencial angle

	int pointXLeftGoal = 0;
	int pointYLeftGoal = 0;

	trigonometricAngle = (pointRightAngleRobot + 90)*M_PI / 180;//angle in radians
	pointXLeftGoal = goalX + int(lineRatio*cos(trigonometricAngle));
	pointYLeftGoal = goalY + int(lineRatio*sin(trigonometricAngle));

	//matrixFillCircleWrite(pointXLeftGoal, lengthField - pointYLeftGoal, 3, 4);
	//-----right point goal-----//
	int pointRightAngleGoal = goalAngle + 90;//tangencial angle

	int pointXRightGoal = 0;
	int pointYRightGoal = 0;

	trigonometricAngle = (pointLeftAngleRobot + 90)*M_PI / 180;//angle in radians
	pointXRightGoal = goalX + int(lineRatio*cos(trigonometricAngle));
	pointYRightGoal = goalY + int(lineRatio*sin(trigonometricAngle));

	//matrixFillCircleWrite(pointXRightGoal, lengthField - pointYRightGoal, 3, 4);
	//-----middle line-----//
	int lineX = robotX - goalX;//X=Xend-Xstart
	int lineY = robotY - goalY;//Y=Yend-Ystart
	float lineHip = sqrt(pow(lineX, 2) + pow(lineY, 2));//diagonal robot->goal
	float newLineHip = 0;
	float relation = 0;

	int contWhile = 0;
	//-----read line-----//
	while ((lineDistance * numberLines <= lineHip) && (contWhile < 300))
	{
		newLineHip = lineDistance * numberLines;
		relation = (newLineHip / lineHip);//relation between diagonal (robot->goal) and (robot->circle)

		int newLineX = relation*lineX;
		int newLineY = relation*lineY;

		//matrixFillCircleWrite(pointXLeftRobot - newLineX, lengthField - (pointYLeftRobot - newLineY), 3, 4);
		//matrixFillCircleWrite(pointXRightRobot - newLineX, lengthField - (pointYRightRobot - newLineY), 3, 4);
		int leftPointX = pointXLeftRobot - newLineX;
		int leftPointY = (pointYLeftRobot - newLineY);
		int rightPointX = pointXRightRobot - newLineX;
		int rightPointY = (pointYRightRobot - newLineY);

		//drawLine(leftPointX, leftPointY, rightPointX, rightPointY, 2);
		int lastValues5[5] = { 0, 0, 0, 0, 0 };
		for (int i = 0; i < 3; i++)
			lastValues5[i] = lastValues5[i + 1];
		lastValues5[4] = matrixLineRead(leftPointX, leftPointY, rightPointX, rightPointY, 1);

		int lastValues5Sum = 0;
		for (int i = 0; i < 5; i++)
			lastValues5Sum += lastValues5[i];

		if (lastValues5Sum > 1)
		{
			int lineHipCircle = lineDistance * (numberLines - lineRatio);//edit if lineDistance!=1
			float relationCircle = (lineHipCircle / lineHip);

			int lineXCircle = relationCircle*lineX;
			int lineYCircle = relationCircle*lineY;

			drawCircle(robotX - lineXCircle, (robotY - lineYCircle), lineRatio, 4);
			CircleAvoidObstacle(robotX - lineXCircle, (robotY - lineYCircle), lineRatio, goalAngle);
			numberLines += 100;
		}
		numberLines++;
		contWhile++;
	}
	if (contWhile >= 300)
		cout << "FIND INFINITY 1876 ";
}
void updateBestPath(int clientID)
{
	int pathPointsSize = 0;
	int bestPossiblePath = 0;//index X of the best possible X(does not hit anything)

	for (int i = 0; bestPath[i] > 0; i++)
	{
		bestPath[i] = 0;
	}
	for (int i = 0; pathPoints[i] > 0; i++)
	{
		pathPointsSize = i + 1;
	}

	if (matrixBigLineRead(robotX, robotY, goalX, goalY, 11, 1, 1) == 0)
	{
		bestPath[0] = goalX;
		bestPath[1] = goalY;
	}
	else
	{
		for (int i = pathPointsSize - 2; i >= 0; i -= 2)
		{
			if (matrixBigLineRead(robotX, robotY, pathPoints[i], pathPoints[i + 1], 15, 1, 1) == 0 && bestPossiblePath == 0)
			{
				bestPossiblePath = i;
			}
		}

		//bestPath[0] = pathPoints[pathPointsSize - 2];
		//bestPath[1] = pathPoints[pathPointsSize-1];
		bestPath[0] = pathPoints[bestPossiblePath];
		bestPath[1] = pathPoints[bestPossiblePath + 1];
	}
}

//--------------------------------------------------------------SIMULATION FUNCTIONS--------------------------------------------------
void searchSimulationValueError()
{
	for (int i = 0; i < 8; i++)
	{
		if (sonarValue(ID, i) < 0)
		{
			cout << "reinicioooou" << endl;
			Start_sonar(ID);
		}
	}
}*/
//-------------------------------------------------------------------THREAD-----------------------------------------------------------

//-----------------------------------------------------------------INT MAIN-----------------------------------------------------------
int main(int argc, char* argv[])
{
	int newPositionX = 90;
	int newPositionY = 120;

	ID = Initial();

	DataBase dataBase;

	Compass compass;
	Motors motors(compass);
	Ultrasonic sonars(dataBase, compass);
	Camera pixyCam(dataBase, compass);
	FieldMatrix fieldMatrix(sonars);
	FieldDraw draw(dataBase,compass, sonars, fieldMatrix);
	Dribble dribble;
	BPSensor BPsensor;
	Solenoid solenoid;

	compass.initializeSimulation(ID);
	motors.initializeSimulation(ID);
	sonars.initializeSimulation(ID);
	pixyCam.initializeSimulation(ID);
	dribble.initializeSimulation(ID);
	BPsensor.initializeSimulation(ID);
	solenoid.initializeSimulation(ID);

	motors.setMaxPower(100);//set maximum motor power
	dribble.setMaxPower(255);//set maximum motor power

	draw.setSeeRobot(true);//able to see the robot
	draw.setSeeBall(true);//able to see the robot
	draw.setSeeSonarLines(true);//able to see sonar lines

	while (true)
	{
		int check = fStart_Stop(ID);
		cout << "a";
		if (ID != -1)
		{
			cout << "b";
			cout << "Servidor conectado!\n" << std::endl;
			system("CLS");

			for (int i = 0; i < 16; i++)//enter to don't write in field pixel
			{
				cout << endl;
			}

			float moveFront = true;//delete this
			//--------------------------------------------------------SIMULATION------------------------------------------------------
			while (simxGetConnectionId(ID) != -1) // while the simulation is active
			{
				//---------------------------------------------------------Drawning Field
				fieldMatrix.drawSonarPoints();//draw sonar points in the field (140ms)->(90ms)->(48ms)->(6-7ms)
				erasePoints.setTimeMilliseconds(3000);//was bugging
				erasePoints.update();
				if (erasePoints.executeThread() == true)
				{
					fieldMatrix.delAlonePoints();//delete alone points in the field
				}

				draw.seeField();//draw field
				//*---------------------------------------------------------Ball
				BPsensor.value();
				//*---------------------------------------------------------Moviment

				/*if (dataBase.getRobotY() > 120 && moveFront == true)
				{
					BPsensor.setRobotWithBall(false);
					solenoid.kick();
					moveFront = false;
				}
				else if (dataBase.getRobotY() < 40 && moveFront == false)
					moveFront = true;*/

				/*if (moveFront == true)
					motors.simpleMove(0, dataBase.getRobotBallAngle());
				else
					motors.simpleMove(180, dataBase.getRobotBallAngle());*/

				/*if (moveFront == true)
					motors.simpleMove(0, 0);
				else
					motors.simpleMove(180, 0);*/
				dribble.motor(-200);
				if (BPsensor.value() == true)
				{
					BPsensor.setRobotWithBall(false);
					extApi_sleepMs(200);
					solenoid.kick();
				}
				

				//motors.simpleMove(-90, 0);

				/*updateGlobalFrontRobotAngle(ID);
				goalX = ballX;
				goalY = ballY;

				generateRoute(ID);
				updateGoalAngle(ID);
				updateBestPath(ID);
				
				goToPosition(ID, bestPath[0], bestPath[1], 0, 200);*/
				///------------------------------------------------------Debug
				//DEBUG 50ms
				debug.setTimeMilliseconds(100); debug.update();
				if (debug.executeThread() == true){
					pixyCam.uptadeXYBall();
					pixyCam.updateServoAngle();

					if (dataBase.getIsBallKnowed() == false)
					{
						if (dataBase.getLastBallWasRight())
						{
							dataBase.setRobotBallAngle(360 - compass.value() - 20);
						}
						else
						{
							dataBase.setRobotBallAngle(360 - compass.value() + 20);
						}
					}
				}
				///------------------------------------------------------Debug1
				//DEBUG 200ms
				debug1.setTimeMilliseconds(2000); debug1.update();
				if (debug1.executeThread() == true){ 
	
					/*int compassValue = compass.value();
					int ballValue = 360-dataBase.getRobotBallAngle();

					while (ballValue < 0)
						ballValue+= 360;
					while (ballValue >360)
						ballValue-= 360;
					while (compassValue < 0)
						compassValue += 360;
					while (compassValue >360)
						compassValue -= 360;

					cout << compassValue - ballValue << " ";*/
				}
				//extApi_sleepMs(5);//delay 5ms
			}
			cout << "THE END";
		}
	}

	return 0;
}


//testVelocity.setPointTime();
//cout << testVelocity.getPointTime() << " ";