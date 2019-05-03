// Library (DataBase) robô de futebol 2018 CIC Robotics
// 18/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#ifndef DATABASE_H
#define DATABASE_H

#if !defined(ARDUINO)
#include <cstdint>//int8_t values
#endif

class DataBase
{
public:
	DataBase();

	void setBallX(int value);//-----------ball
	int getBallX() const;
	void setBallY(int value);
	int getBallY() const;
	void setIsBallKnowed(bool value);
	bool getIsBallKnowed() const;
	float getBallDiameter() const;
	void setLastBallWasRight(bool value);
	bool getLastBallWasRight();


	void setRobotBallAngle(int value);//----------robotAngle
	int getRobotBallAngle() const;

	void setRobotX(int value);//----------robotX
	int getRobotX() const;
	void setIsRobotXKnowed(bool value);
	bool getIsRobotXKnowed() const;

	void setRobotY(int value);//----------robotY
	int getRobotY() const;
	void setIsRobotYKnowed(bool value);
	bool getIsRobotYKnowed() const;

	int getWidthField() const;//----------field
	int getLengthField() const;

private:
	//ball
	int ballX;
	int ballY;
	bool isBallKnowed;
	const float ballDiameter;
	bool lastBallWasRight;
	//robot
	int robotX;
	bool isRobotXKnowed;
	int robotY;
	bool isRobotYKnowed;
	int robotBallAngle;
	//friend robot
	int friendRobotX;
	int friendRobotY;
	bool isFriendRobotKnowed;
	//adversary robot
	uint8_t obstaclesReadXY[24];//X,Y sonar obstacle reading
	uint8_t adversaryXY[24];
	bool isAdversaryXYKnowed;
	//field
	const int widthField;
	const int lengthField;
};

#endif // DATABASE_H