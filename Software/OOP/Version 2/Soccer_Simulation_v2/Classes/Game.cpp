///////////////////////////////////////////////////////////
//  Game.cpp
//  Implementation of the Class Game
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////
#define DEBUG 0

#include "Game.h"

#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;
#include <iomanip>//setprecision
using std::setprecision;

extern "C" {
#include "extApi.h"
}

Game::Game(Robot *_robot0, Robot *_robot1, Robot *_robot2, Robot *_robot3)
	:realBallX(0), realBallY(0), clientIDSimulation(0),
	robots(vector <Robot*>(4))
{
	robot0 = _robot0; robots[0] = robot0;
	robot1 = _robot1; robots[1] = robot1;
	robot2 = _robot2; robots[2] = robot2;
	robot3 = _robot3; robots[3] = robot3;

	timeWithoutBall = new TimeThread(500);
}

Game::~Game(){

}

void Game::execute()
{
	static bool firstGame = true;

	int ret = 0;
	int message = -1;
	ret = simxGetInMessageInfo(clientIDSimulation, simx_headeroffset_server_state, &message);
	if ((message & 1) == 0){//if it has been stopped
		message = -1;
		ret = simxGetInMessageInfo(clientIDSimulation, simx_headeroffset_server_state, &message);
		float ballPos[3] = { 0, 0, 0.1 };//SimulationSuctionPadHandle{ 0.120, -0.086, 0 }
		simxSetObjectPosition(clientIDSimulation, handleBall, -1, ballPos, simx_opmode_oneshot);
	}
	if ((message & 2) == 2){//if it has been paused
		message = -1;
		ret = simxGetInMessageInfo(clientIDSimulation, simx_headeroffset_server_state, &message);
	}

	updateRealXYRobots();
	updateRealXYBall();
	updatePositionBall();

	cout << "Real ball position:		(" << std::setw(6) << std::left << 182-realBallX << "," << std::setw(6) << 243-realBallY << ")" << endl;

	if (realBallX < 30 || realBallX>152 || realBallY < 30 || realBallY>213){
		//cout << "BALL OUT";
		float ballPos[3] = {0, 0, 0.1 };//SimulationSuctionPadHandle{ 0.120, -0.086, 0 }
		simxSetObjectPosition(clientIDSimulation, handleBall, -1, ballPos, simx_opmode_oneshot);
	}

	//cout << endl;
	if (DEBUG){
		printf("--------------- Game Manager ---------------\n");
		for (int i = 0; i < 4; i++){
			if (robotConnected[i] == true){
				printf("Robot%d: (%0.1f,%0.1f)\n", i, realRobotX[i], realRobotY[i]);
			}
		}

		printf("Ball: (%0.1f,%0.1f)\n", realBallX, realBallY);

		cout << endl;
	}
}
void Game::updateRealXYBall()
{
	if (ballConnected == true)
	{
		float posSimulation[3] = { 0 };

		simxGetObjectPosition(clientIDSimulation, handleBall, -1, posSimulation, simx_opmode_streaming);

		realBallX = 91 + posSimulation[0] * 100;
		realBallY = 121.5 + posSimulation[1] * 100;
	}
}
void Game::updateRealXYRobots()
{
	for (int i = 0; i < 4; i++)
	{

			if (robotConnected[i] == true)
			{
				float posSimulation[3] = { 0 };

				simxGetObjectPosition(clientIDSimulation, handleRobots[i], -1, posSimulation, simx_opmode_streaming);

				realRobotX[i] =91 - posSimulation[0] * 100;
				realRobotY[i] =121.5 - posSimulation[1] * 100;
			}
		}
}

void Game::updatePositionBall()
{
		for (int i = 0; i < 4; i++)
		{
			if (robotConnected[i] == true)
			{
				if (robots[i]->GetRobotWithBall() == true)
				{
					rWithBall[i] = true;
				}

				if ((robots[i]->GetRobotWithBall() == false) && (rWithBall[i] == true));
				{
					if (robots[i]->kicked==true)
						rWithBall[i] = false;
					//else
						//rWithBall[i] = true;
				}

				if (rWithBall[i] == true)
				{
					robots[i]->SetRobotWithBall(true);
					//float ballPos[3] = { 0.120, -0.076, 0 };
					//simxSetObjectPosition(clientIDSimulation, handleBall, handleRobots[i], ballPos, simx_opmode_oneshot);
				}
			}
		}
}

void Game::initializeSimulation(int ID){
	//handle robots
	for (int i = 0; i < 4; i++)
	{
		handleRobots[i] = robots[i]->GetSRobotHandle();
	}
	//robots connected
	for (int i = 0; i < 4; i++)
	{
		if (handleRobots[i] == 0)
			robotConnected[i] = false;
		else
			robotConnected[i] = true;
	}
	//robots without ball
	for (int i = 0; i < 4; i++)
	{
		rWithBall[i] = false;
	}
	//robot X
	for (int i = 0; i < 4; i++)
	{
		realRobotX[i] = 0;
	}
	//robot Y
	for (int i = 0; i < 4; i++)
	{
		realRobotY[i] = 0;
	}

	cout << "---------- Connecting Game Manager ----------" << endl;
	clientIDSimulation = ID;
	//Handle ball
	if (simxGetObjectHandle(clientIDSimulation, "Ball", &handleBall, simx_opmode_blocking) != simx_return_ok){
		cout << "Ball not found!" << endl;
		ballConnected = false;
	}
	else{
		ballConnected = true;
		cout << "Connected to the Ball" << endl;
	}
	//Robots
	cout << "Connected to the robots: ";
	for (int i = 0; i < 4; i++)
	{
		if (robotConnected[i] == true)
			cout << "R" << i << " ";
	}
}

//---------- Outputs ----------//

void Game::getXYRobotNeuralNet(vector<float> &robotXY, int robotNum){
	robotXY.clear();
	robotXY.push_back((realRobotX[robotNum])/243);
	robotXY.push_back((realRobotY[robotNum])/243);
}