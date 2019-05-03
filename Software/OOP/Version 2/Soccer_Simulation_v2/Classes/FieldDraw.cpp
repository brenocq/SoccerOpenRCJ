///////////////////////////////////////////////////////////
//  FieldDraw.cpp
//  Implementation of the Class FieldDraw
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "FieldDraw.h"
#include "Robot.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

#include <cstdint>//int16_t values
#include <algorithm>//swap function
using std::swap;
#include <string>//string
#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;
#include <cstdlib>//to clear the screen

FieldDraw::FieldDraw(Robot *_robot1, Robot *_robot2, Robot *_robot3, Robot *_robot4)
	:robot1(*_robot1), robot2(*_robot2), robot3(*_robot3), robot4(*_robot4),
	robots(vector <Robot*>(5)),
	firstPrint(true)
{
	robots[1] = &robot1;
	robots[2] = &robot2;
	robots[3] = &robot3;
	robots[4] = &robot4;

	for (int i = 0; i <= 4; i++)
	{
		seeRobot[i] = EMPTY;
		seeSonarLines[i] = EMPTY;
		seeReading[i] = EMPTY;
		seeBall[i] = EMPTY;
		seeAdversaries[i] = EMPTY;
		seeFriend[i] = EMPTY;
		seePath[i] = EMPTY;
	}


	fieldMatrix = new Color*[widthField];
	for (int i = 0; i < widthField; ++i)
		fieldMatrix[i] = new Color[lengthField];//generate field

	cleanField(WHITE);

	myconsole = GetConsoleWindow();
	mydc = GetDC(myconsole);
}
FieldDraw::~FieldDraw(){
	for (int i = 0; i < widthField; ++i) {
		delete[] fieldMatrix[i];
	}
	delete[] fieldMatrix;
}

//----- Robots -----//

//----- Read -----//
int FieldDraw::pointRead(int x0, int y0, int val)
{
	if (fieldMatrix[x0][y0] == val)
		return 1;
	else
		return 0;
}

//----- Write -----//
void FieldDraw::pointWrite(int x0, int y0, int val)
{
	x0 >= 182 ? x0-- : x0;
	y0 >= 243 ? y0-- : y0;

	Color color = static_cast<Color>(val);
	fieldMatrix[x0][y0] = color;
}

//----- Configure Draw -----//
void FieldDraw::setSeeRobot(bool b, int robotNumber, Color color){
	if (b == true)
		seeRobot[robotNumber] = color;
	else
		seeRobot[robotNumber] = EMPTY;
}
void FieldDraw::setSeeSonarLines(bool b, int robotNumber, Color color){
	if (b == true)
		seeSonarLines[robotNumber] = color;
	else
		seeSonarLines[robotNumber] = EMPTY;
}
void FieldDraw::setSeeSonarPoints(bool b, int robotNumber, Color color)
{
	if (b == true)
		seeSonarPoints[robotNumber] = color;
	else
		seeSonarPoints[robotNumber] = EMPTY;
}
void FieldDraw::setSeeReading(bool b, int robotNumber, Color color){
	if (b == true)
		seeReading[robotNumber] = color;
	else
		seeReading[robotNumber] = EMPTY;
}
void FieldDraw::setSeeBall(bool b, int robotNumber, Color color){
	if (b == true)
		seeBall[robotNumber] = color;
	else
		seeBall[robotNumber] = EMPTY;
}
void FieldDraw::setSeeAdversaries(bool b, int robotNumber, Color color){
	if (b == true)
		seeAdversaries[robotNumber] = color;
	else
		seeAdversaries[robotNumber] = EMPTY;
}
void FieldDraw::setSeeFriend(bool b, int robotNumber, Color color){
	if (b == true)
		seeFriend[robotNumber] = color;
	else
		seeFriend[robotNumber] = EMPTY;
}
void FieldDraw::setSeePath(bool b, int robotNumber, Color color){
	if (b == true)
		seePath[robotNumber] = color;
	else
		seePath[robotNumber] = EMPTY;
}

void FieldDraw::setSeeFieldLine(bool b, Color color){
	if (b == true)
		seeFieldLine = color;
	else
		seeFieldLine = EMPTY;
}

//draw
void FieldDraw::promptSetPixel(int x, int y, COLORREF color){
	SetPixel(mydc, x, y, color);
	/*SetPixel(mydc, x * 2, y * 2, color);
	SetPixel(mydc, x * 2 + 1, y * 2, color);
	SetPixel(mydc, x * 2, y * 2 + 1, color);
	SetPixel(mydc, x * 2 + 1, y * 2 + 1, color);*/
}


void FieldDraw::print(){
	

	COLORREF white = RGB(255, 255, 255);
	COLORREF green = RGB(0, 255, 0);
	COLORREF red = RGB(255, 0, 0);
	COLORREF blue = RGB(0, 0, 255);
	COLORREF magenta = RGB(255, 0, 255);
	COLORREF purple = RGB(128, 0, 128);
	COLORREF black = RGB(0, 0, 0);
	COLORREF cyan = RGB(0, 255, 255);
	COLORREF yellow = RGB(255, 255, 0);
	COLORREF orange = RGB(255, 69, 0);

	//arcThreePoints(30, 50, 91, 80, 152, 50, orange);

	for (int i = 0; i < lengthField; i++)//Y
	{
		for (int j = 0; j < widthField; j++)//X
		{
			switch (fieldMatrix[j][i])
			{
			case EMPTY:
				break;
			case WHITE:
				promptSetPixel(j, lengthField - i, white);
				fieldMatrix[j][i] = EMPTY;
				break;
			case GREEN:
				promptSetPixel( j, lengthField - i, green);
				fieldMatrix[j][i] = WHITE;
				break;
			case RED:
				promptSetPixel(j, lengthField - i, red);
				fieldMatrix[j][i] = WHITE;
				break;
			case BLUE:
				promptSetPixel( j, lengthField - i, blue);
				fieldMatrix[j][i] = WHITE;
				break;
			case MAGENTA:
				promptSetPixel(j, lengthField - i, magenta);
				fieldMatrix[j][i] = WHITE;
				break;
			case PURPLE:
				promptSetPixel( j, lengthField - i, purple);
				fieldMatrix[j][i] = WHITE;
				break;
			case BLACK:
				promptSetPixel( j, lengthField - i, black);
				fieldMatrix[j][i] = WHITE;
				break;
			case CYAN:
				promptSetPixel(j, lengthField - i, cyan);
				fieldMatrix[j][i] = WHITE;
				break;
			case YELLOW:
				promptSetPixel( j, lengthField - i, yellow);
				fieldMatrix[j][i] = WHITE;
				break;
			case ORANGE:
				promptSetPixel(j, lengthField - i, orange);
				fieldMatrix[j][i] = WHITE;
				break;
			}
		}
	}

	if (seeFieldLine != EMPTY)
	{
		lineWrite(30, 30,30,213, seeFieldLine);
		lineWrite(152, 30, 152, 213, seeFieldLine);
		lineWrite(30, 30, 152, 30, seeFieldLine);
		lineWrite(30, 213, 152, 213, seeFieldLine);
	}

	for (int i = 1; i <= 4; i++)
	{
		if (seeRobot[i] != EMPTY)
		{
			bool isRobotXYKnowed = robots[i]->GetIsRobotXKnowed() && robots[i]->GetIsRobotYKnowed();
			int robotX = robots[i]->GetRobotX();
			int robotY = robots[i]->GetRobotY();
			int robotRatio = robots[i]->GetRobotRatio();
			int ballDiameter = robots[i]->GetBallDiameter();

			if (i % 2 == 0)//******************************************ROBOT 1 AND 3************************************************
			{
				if (seeReading[i] != EMPTY)//-----SEE READING-----//
				{
					for (int j = 0; j < lengthField; j++)//Y
					{
						for (int k = 0; k < widthField; k++)//X
						{
							if (robots[i]->fieldMatrix->pointRead(k, j, 1) == 1)
							{
								pointWrite(widthField - 1 - k, lengthField - 1 - j, seeReading[i]);
							}
						}
					}
				}
				if (seePath[i] != EMPTY)//-----SEE PATH-----//
				{
					int pathSize = robots[i]->pathRobot.size() / 2;
					for (int p = 1; p <= pathSize; p++){
						int pathX = robots[i]->pathRobot[(p * 2) - 2];
						int pathY = robots[i]->pathRobot[(p * 2) - 1];
						//fillCircleWrite(widthField - 1 - pathX, lengthField - 1 - pathY, 3, seePath[i]);
						fillCenterSquareWrite(widthField - 1 - pathX, lengthField - 1 - pathY, 2, 2, seePath[i]);
					}
				}
				if (seeFriend[i] != EMPTY)//-----SEE FRIEND-----//
				{
					if (robots[i]->GetIsFriendKnowed())
					{
						int *friendXY = robots[i]->GetFriendXY();
						fillCircleWrite(widthField - 1 - friendXY[0], lengthField - 1 - friendXY[1], robotRatio, seeAdversaries[i]);
					}
				}
				if (isRobotXYKnowed)//-----if is robot X and Y knowed-----//
				{//-----SEE ROBOT XY-----//
					fillCircleWrite(widthField - 1 - robotX, lengthField - 1 - robotY, robotRatio, seeRobot[i]);
					if (seeSonarLines[i] != EMPTY)//-----SEE SONAR LINES-----//
					{
						for (int j = 0; j < 12; j++)
						{
							int pointSonarX = robots[i]->sonarPointX[j] + robotX;
							int pointSonarY = robots[i]->sonarPointY[j] + robotY;
							lineWrite(widthField - 1 - robotX, lengthField - 1 - robotY, widthField - 1 - pointSonarX, lengthField - 1 - pointSonarY, seeSonarLines[i]);
						}
					}
					if (seeSonarPoints[i] != EMPTY)//-----SEE SONAR POINTS-----//
					{
						for (int j = 0; j < 12; j++)
						{
							int pointSonarX = robots[i]->sonarPointX[j] + robotX;
							int pointSonarY = robots[i]->sonarPointY[j] + robotY;
							fillCircleWrite(widthField - 1 - pointSonarX, lengthField - 1 - pointSonarY, 2, seeSonarLines[i]);
						}
					}
					if (seeAdversaries[i] != EMPTY)//-----SEE RELATIVE ADVERSARIES-----//
					{
						if (robots[i]->GetNumAdversaries()>0)
						{
							int *adversariesXY = robots[i]->GetAdversaryXY();
							for (int j = 1; j <= robots[i]->GetNumAdversaries(); j++){
								int advX = adversariesXY[(j - 1) * 2 + 1];
								int advY = adversariesXY[(j - 1) * 2 + 2];
								fillCircleWrite(widthField - 1 - advX, lengthField - 1 - advY, 4, seeAdversaries[i]);
							}
						}
					}
					if (seeBall[i] != EMPTY)//-----SEE RELATIVE ROBOT BALL-----//
					{
						if (robots[i]->GetIsBallKnowed())
						{
							int ballX = robots[i]->GetBallX();
							int ballY = robots[i]->GetBallY();
							fillCircleWrite(widthField - 1 - ballX, lengthField - 1 - ballY, ballDiameter, seeBall[i]);
						}
					}
				}
			}
			else//***************************************************ROBOT 0 AND 2***************************************************
			{
				if (seeReading[i] != EMPTY)//-----SEE READING-----//
				{
					for (int j = 0; j < lengthField; j++)//Y
					{
						for (int k = 0; k < widthField; k++)//X
						{
							if (robots[i]->fieldMatrix->pointRead(k, j, 1) == 1)
							{
								pointWrite(k, j, seeReading[i]);
							}
						}
					}
				}
				if (seePath[i] != EMPTY)//-----SEE PATH-----//
				{
					int pathSize = robots[i]->pathRobot.size() / 2;
					for (int p = 1; p <= pathSize; p++){
						int pathX = robots[i]->pathRobot[(p * 2) - 2];
						int pathY = robots[i]->pathRobot[(p * 2) - 1];
						//fillCircleWrite(pathX,pathY, 3, seePath[i]);
						fillCenterSquareWrite(pathX, pathY, 2, 2, BLUE);
						//circleWrite(pathX, pathY, 17, 1, seePath[i]);
						//fillCenterSquareWrite(60, 120, 7, 7, BLUE);
						bigLineWrite(55, 115, 65, 125, 2, BLUE);
						bigLineWrite(55, 125, 65, 115, 2, BLUE);
					}
				}
				if (seeFriend[i] != EMPTY)//-----SEE FRIEND-----//
				{
					if (robots[i]->GetIsFriendKnowed())
					{
						int *friendXY = robots[i]->GetFriendXY();
						fillCircleWrite(friendXY[0], friendXY[1], robotRatio, seeAdversaries[i]);
					}
				}
				if (isRobotXYKnowed)//-----if is robot X and Y knowed-----//
				{
					fillCircleWrite(robotX, robotY, robotRatio, seeRobot[i]);
					//fillCircleWrite(robotX,robotY, 1, GREEN);
					if (seeSonarLines[i] != EMPTY)//-----SEE SONAR LINES-----//
					{
						for (int j = 0; j < 12; j++)
						{
							int pointSonarX = robots[i]->sonarPointX[j] + robotX;
							int pointSonarY = robots[i]->sonarPointY[j] + robotY;
							lineWrite(robotX, robotY, pointSonarX, pointSonarY, seeSonarLines[i]);
						}
					}
					if (seeSonarPoints[i] != EMPTY)//-----SEE SONAR POINTS-----//
					{
						for (int j = 0; j < 12; j++)
						{
							int pointSonarX = robots[i]->sonarPointX[j] + robotX;
							int pointSonarY = robots[i]->sonarPointY[j] + robotY;
							fillCircleWrite(pointSonarX, pointSonarY, 2, seeSonarLines[i]);
						}
					}
					if (seeAdversaries[i] != EMPTY)//-----SEE RELATIVE ADVERSARIES-----//
					{
						if (robots[i]->GetNumAdversaries()>0)
						{
							int *adversariesXY = robots[i]->GetAdversaryXY();
							for (int j = 1; j <= robots[i]->GetNumAdversaries(); j++){
								int advX = adversariesXY[(j - 1) * 2 + 1];
								int advY = adversariesXY[(j - 1) * 2 + 2];
								fillCircleWrite(advX, advY, 4, seeAdversaries[i]);
							}
						}
					}
					if (seeBall[i] != EMPTY)//-----SEE RELATIVE ROBOT BALL-----//
					{
						if (robots[i]->GetIsBallKnowed())
						{
							int ballX = robots[i]->GetBallX();
							int ballY = robots[i]->GetBallY();
							fillCircleWrite(ballX, ballY, ballDiameter, seeBall[i]);
						}
					}
				}
			}
		}
	}
}
