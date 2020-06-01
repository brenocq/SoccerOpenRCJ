#include "stdafx.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <windows.h>
#include "Classes/Robot.h"
#include "Classes/Game.h"
#include "Classes/FieldDraw.h"
#include "Classes/TimeThread.h"
#include "Classes/NeuralNetwork.h"
#include "Classes/ExternalCommunication.h"
extern "C" {
#include "CopelliaSim/extApi.h"
}
using namespace std;

auto EMPTY = FieldDraw::EMPTY;
auto WHITE = FieldDraw::WHITE;
auto BLACK = FieldDraw::BLACK;
auto RED = FieldDraw::RED;
auto GREEN = FieldDraw::GREEN;
auto BLUE = FieldDraw::BLUE;
auto MAGENTA = FieldDraw::MAGENTA;
auto PURPLE = FieldDraw::PURPLE;
auto CYAN = FieldDraw::CYAN;
auto YELLOW = FieldDraw::YELLOW;
auto ORANGE = FieldDraw::ORANGE;

auto ATTACKER = Robot::ATTACKER;
auto DEFENDER = Robot::DEFENDER;
//---------- DEFINES ----------//
#define DEBUG 1//1= show debug - 0= show only robots

//-----------------------------------------------------------GLOBAL VARIABLES---------------------------------------------------------


//----------------------------------------------------------SIMULATION CONTROL--------------------------------------------------------
int Initial()
{
	simxChar *DommTest;
	DommTest = "Dummy";
	simxInt *check = new simxInt[1];
	check[0] = 0;
	check[1] = 0;
	int portNb = 19997;
	int clientID = -1;
	simxFinish(-1);

	clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);

	if (clientID > -1)
	{

		simxGetObjectHandle(clientID, DommTest, check, simx_opmode_oneshot_wait);
		printf("Connection Success ... \n");
		printf("\n Number of ID:  %d", clientID);
		printf("\n check: %d\n\n", check);
		simxStopSimulation(clientID, simx_opmode_oneshot);
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
	//char Star_Stop = getchar();
	printf("\n");
	int start = -1;
	if (true)//(Star_Stop == 'Y')
	{
		printf("Starting ...\n");
		start = simxStartSimulation(ID, simx_opmode_oneshot);
	}
	else //if (Star_Stop == 'N')
	{
		printf("Closing...\n");
		start = simxStopSimulation(ID, simx_opmode_oneshot);
	}
	system("CLS");
	return start;
}
//---------------------------------------------------------------OBJECTS--------------------------------------------------------------

// ----- THREADS-----//
TimeThread checkTime(0);
TimeThread debug1(500);//create debug1 thread
TimeThread debug2(1000);//create debug2 thread
TimeThread debug3(5000);//create debug3 thread
TimeThread debugErase(200);//create thread erase
//----- ROBOTS -----//
int ID = Initial();
Robot *robot0();
Robot *robot1();
Robot *robot2();
Robot *robot3();
//----- FIELD -----//
FieldDraw *field();
//----- GAMECONTROL -----//
Game *gameControl();

//----------------------------------------------------------------VOIDS---------------------------------------------------------------
//----- INITIALIZATION -----//
void initializateObjects(){
	//----- SIMULATION -----//
	ID = Initial();
	cout << fixed << setprecision(2);//configure float output as 0.00

	int check = fStart_Stop(ID);
}
//----- PRINT -----//
void printClean()
{
	COORD pos = { 0, 14 };
	HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleCursorPosition(output, pos);
	if (debugErase.executeThread())//-----100ms
	{
		/*for (int i = 0; i < 30; i++)
			cout << "                                                                      " << endl;*/
		SetConsoleCursorPosition(output, pos);
	}
}
//-----------------------------------------------------------------MAIN---------------------------------------------------------------
//-----------------------------------------------------------------MAIN---------------------------------------------------------------
//-----------------------------------------------------------------MAIN---------------------------------------------------------------
int main(int argc, char* argv[])
{
	HWND hwnd = GetConsoleWindow();
	MoveWindow(hwnd, 0, 0, 700, 830, TRUE);//xpos,ypos,xsize,ysize

	ExternalCommunication bluetooth1(0, 2);//communication between robots 0 and 2
	//ExternalCommunication bluetooth2(1, 3);//communication between robots 1 and 3

	/*Robot *robot0 = new Robot(0, ID, &bluetooth1, ATTACKER);
	Robot *robot1 = new Robot(1, ID, &bluetooth2, DEFENDER);
	Robot *robot2 = new Robot(2, ID, &bluetooth1, DEFENDER);
	Robot *robot3 = new Robot(3, ID, &bluetooth2, ATTACKER);*/

	Robot *robot0 = new Robot(0, ID, &bluetooth1, ATTACKER);
	Robot *robot1 = new Robot(1, ID, 0, DEFENDER);
	Robot *robot2 = new Robot(2, ID, &bluetooth1, DEFENDER);
	Robot *robot3 = new Robot(3, ID, 0, ATTACKER);

	FieldDraw *field = new FieldDraw(robot0, robot1, robot2, robot3);
	field->setSeeFieldLine(true, BLACK);

	field->setSeeRobot(true, 1, BLUE);
	field->setSeeSonarLines(true, 1, BLUE);
	field->setSeeSonarPoints(true, 1, BLUE);
	field->setSeeReading(true, 1, RED);
	field->setSeeBall(true, 1, MAGENTA);
	field->setSeePath(true, 1, GREEN);
	field->setSeeAdversaries(false, 1, MAGENTA);

	field->setSeeRobot(true, 2, ORANGE);
	field->setSeeSonarLines(true, 2, ORANGE);
	field->setSeeSonarPoints(true, 2, ORANGE);
	field->setSeeReading(true, 2, ORANGE);
	field->setSeeBall(true, 2, ORANGE);
	field->setSeePath(true, 2, ORANGE);

	field->setSeeRobot(false, 3, PURPLE);
	field->setSeeSonarLines(false, 3, PURPLE);
	field->setSeeSonarPoints(false, 3, PURPLE);
	field->setSeeReading(false, 3, BLUE);
	field->setSeeBall(false, 3, BLUE);
	field->setSeePath(true, 3, BLUE);
	Game *gameControl = new Game(robot0, robot1, robot2, robot3);
	gameControl->initializeSimulation(ID);
	initializateObjects();
	system("CLS");
	while (ID != -1){
			//-----Print-----//
			printClean();
			field->print();
			//-----Game Control-----//
			gameControl->execute();//                robot judge / ball judge
			//-----Robots-----//
			robot0->execute();
			robot1->execute();
			//robot2->execute();


			if (DEBUG == 1){
				if (debug1.executeThread()){// 500ms
					
				}
				if (debug2.executeThread()){// 1000ms

				}
				if (debug3.executeThread()){// 5000ms

				}
		}
		extApi_sleepMs(5);
	}
	cout << "Simulation stopped";
	return 0;
}
//---------- ANNOTATIONS ----------//
//checkTime.defineTimePoint();
//cout << checkTime.getTimePoint() << " ";
