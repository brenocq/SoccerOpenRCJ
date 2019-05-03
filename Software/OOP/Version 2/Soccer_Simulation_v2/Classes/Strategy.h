///////////////////////////////////////////////////////////
//  Strategy.h
//  Implementation of the Class Strategy
//  Created on:      21-mar-2018 21:46:59
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef STRATEGY_H
#define STRATEGY_H

class Strategy
{

public:
	Strategy();
	virtual ~Strategy();

	void updateRobotValues(int X, int Y);
	void updateBallValues(int X, int Y, bool b);
	void updateAdversariesValues(int X1, int Y1, int X2, int Y2);
	void updateFriendXY(int X, int Y);
	void chooseStrategy();

private:
	int robotXY;
	int friendXY;
	int adversary1XY;
	int adversary2XY;
	int ballXY;
	bool ballPossession;

};
#endif // STRATEGY_H
