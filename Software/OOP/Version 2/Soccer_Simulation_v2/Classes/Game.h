///////////////////////////////////////////////////////////
//  Game.h
//  Implementation of the Class Game
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef GAME_H
#define GAME_H

#include "TimeThread.h"
#include "Robot.h"
#include "FieldDraw.h"

#include <vector>
using std::vector;

class Game
{

public:
	Game(Robot *_robot0, Robot *_robot1, Robot *_robot2, Robot *_robot3);
	virtual ~Game();

	void execute();
	//----- Update -----//
	void updateRealXYBall();
	void updateRealXYRobots();
	void updatePositionBall();
	//----- Outputs -----//
	void getXYRobotNeuralNet(vector<float> &robotXY, int robotNum);

	//Simulation
	void initializeSimulation(int ID);
private:
	//Robots
	bool robotConnected[4];
	Robot *robot0; Robot *robot1; Robot *robot2; Robot *robot3;
	vector <Robot*> robots;
	int handleRobots[4];
	float realRobotX[4];
	float realRobotY[4];
	bool rWithBall[4];//used because if the robot lost the ball a short time ago 
					  //is necessary put it more distant to don't read withBall again and ajust the ball to the robot
	//Ball
	int handleBall;
	bool ballConnected;
	float realBallX;
	float realBallY;
	//TimeThread
	TimeThread *timeWithoutBall;
	//Simulation
	int clientIDSimulation;

};
#endif // GAME_H
