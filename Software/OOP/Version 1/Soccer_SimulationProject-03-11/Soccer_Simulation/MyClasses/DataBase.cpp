// Library (DataBase) robô de futebol 2018 CIC Robotics
// 17/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "DataBase.h"

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else
#include <cstdint>//int16_t values
#endif

DataBase::DataBase()
	:ballX(0), ballY(0), isBallKnowed(false), ballDiameter(6.5),
	robotX(0),isRobotXKnowed(false),
	robotY(0), isRobotYKnowed(false),
	friendRobotX(0), friendRobotY(0), isFriendRobotKnowed(false),
	widthField(182), lengthField(243), isAdversaryXYKnowed(false)
{
	for (int i = 0; i < 24; i++)
	{
		obstaclesReadXY[i] = 0;
	}
	for (int i = 0; i < 24; i++)
	{
		adversaryXY[i] = 0;
	}
}

//-----BALL-----//
void DataBase::setBallX(int value)
{
	ballX = value;
}
int DataBase::getBallX() const
{
	return ballX;
}
void DataBase::setBallY(int value)
{
	ballY = value;
}
int DataBase::getBallY() const
{
	return ballY;
}
void DataBase::setIsBallKnowed(bool value)
{
	isBallKnowed = value;
}
bool DataBase::getIsBallKnowed() const
{
	return isBallKnowed;
}
void DataBase::setLastBallWasRight(bool value)
{
	lastBallWasRight = value;
}
bool DataBase::getLastBallWasRight()
{
	return lastBallWasRight;
}

float DataBase::getBallDiameter() const
{
	return ballDiameter;
}

//-----Robot-----//
void DataBase::setRobotBallAngle(int value)
{
	robotBallAngle = value;
}
int DataBase::getRobotBallAngle() const
{
	return robotBallAngle;
}

void DataBase::setRobotX(int value)
{
	robotX = value;
}
int DataBase::getRobotX() const
{
	return robotX;
}
void DataBase::setIsRobotXKnowed(bool value)
{
	isRobotXKnowed = value;
}
bool DataBase::getIsRobotXKnowed() const
{
	return isRobotXKnowed;
}

void DataBase::setRobotY(int value)
{
	robotY = value;
}
int DataBase::getRobotY() const
{
	return robotY;
}
void DataBase::setIsRobotYKnowed(bool value)
{
	isRobotYKnowed = value;
}
bool DataBase::getIsRobotYKnowed() const
{
	return isRobotYKnowed;
}

//-----Friend-----//
//-----Field-----//
int DataBase::getWidthField() const
{
	return widthField;
}
int DataBase::getLengthField() const
{
	return lengthField;
}