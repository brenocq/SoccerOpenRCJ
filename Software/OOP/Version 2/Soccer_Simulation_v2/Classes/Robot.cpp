///////////////////////////////////////////////////////////
//  Robot.cpp
//  Implementation of the Class Robot
//  Created on:      21-mar-2018 21:46:58
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#define DEBUG 0
#define DEBUGMOV 0//(debug simpleMove)
#define DEBUGROBOTS 1//(debug robots)

#include "Robot.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else

#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;
#include <iomanip>//setprecision
using std::setprecision;
#include <stdio.h>//printf
#include <time.h>

#include <string>//string
using std::string;

extern "C" {
#include "extApi.h"
}
#endif

#if defined(ARDUINO)
Robot::Robot(int number)
	:robotNumber(number), robotRatio(11), diameter(6.5), clientIDSimulation(0), Kp(0.025), maxPower(255),
	motors(vector <Motor*>(4))
{
	//motor
	motorFR = new Motor(pin, pin);
	motorBR = new Motor(pin, pin);
	motorBL = new Motor(pin, pin);
	motorFL = new Motor(pin, pin);
	motors[0] = motorFR;
	motors[1] = motorBR;
	motors[2] = motorBL;
	motors[3] = motorFL;
	setMaxMotorsPower(255);
	//compass
	compass = new Compass(pin, pin);

}
#else
Robot::Robot(int number, int clientID, ExternalCommunication *_bluetooth,RobotMode _mode)
	:robotNumber(number), robotRatio(11), frontAngle(0), robotWithBall(false), teamWithBall(false), robotMode(_mode),
	ballDiameter(3.25), isBallRight(true), bluetooth(_bluetooth), myPassState(0), friendPassState(0),
	clientIDSimulation(0), SimulationRobotHandle(0),
	Kp(0.15), Ki(0), Kd(0.000), P(0), I(0), D(0), lastPIDprocess(clock()), lastDegree(0), isFirstPID(true),
	maxPower(255),//start motor variables
	servoAngle(60),//servo starts with 60 degrees
	compassValue(0),
	spinTimesSonar(0), numAdversaries(0),
	isFirstRobotX(true), isRobotXKnowed(false), discrepancyX(10),
	isFirstRobotY(true), isRobotYKnowed(false), discrepancyY(10),
	isGoalKnowed(false), realGoalHeight(10), goalHeightCam(0),
	motors(vector <Motor*>(4)), ultrasonics(vector <Ultrasonic*>(12))
{
	//Motors
	motorFR = new Motor("FR", robotNumber);			motors[0] = motorFR;
	motorBR = new Motor("BR", robotNumber);			motors[1] = motorBR;
	motorBL = new Motor("BL", robotNumber);			motors[2] = motorBL;
	motorFL = new Motor("FL", robotNumber);			motors[3] = motorFL;
	motorXicoR = new Motor("XicoL", robotNumber);
	
	//Servo
	servo = new Servo("Servo", robotNumber);

	//Ultrasonics
	ultrasonic1 = new Ultrasonic(1, robotNumber);	ultrasonics[0] = ultrasonic1;
	ultrasonic2 = new Ultrasonic(2, robotNumber);	ultrasonics[1] = ultrasonic2;
	ultrasonic3 = new Ultrasonic(3, robotNumber);	ultrasonics[2] = ultrasonic3;
	ultrasonic4 = new Ultrasonic(4, robotNumber);	ultrasonics[3] = ultrasonic4;
	ultrasonic5 = new Ultrasonic(5, robotNumber);	ultrasonics[4] = ultrasonic5;
	ultrasonic6 = new Ultrasonic(6, robotNumber);	ultrasonics[5] = ultrasonic6;
	ultrasonic7 = new Ultrasonic(7, robotNumber);	ultrasonics[6] = ultrasonic7;
	ultrasonic8 = new Ultrasonic(8, robotNumber);	ultrasonics[7] = ultrasonic8;
	ultrasonic9 = new Ultrasonic(9, robotNumber);	ultrasonics[8] = ultrasonic9;
	ultrasonic10 = new Ultrasonic(10, robotNumber);	ultrasonics[9] = ultrasonic10;
	ultrasonic11 = new Ultrasonic(11, robotNumber);	ultrasonics[10] = ultrasonic11;
	ultrasonic12 = new Ultrasonic(12, robotNumber);	ultrasonics[11] = ultrasonic12;

	for (int i = 0; i < 12; i++)
	{
		sonarValues[i] = 0;
	}
	for (int i = 0; i < 12; i++)
	{
		sonarPointX[i] = 0;
	}
	for (int i = 0; i < 12; i++)
	{
		sonarPointY[i] = 0;
	}

	//Compass
	compass = new Compass("Compass", robotNumber);
	
	//LightSensor
	lightBallPos = new LightSensor("LS_BallPos", robotNumber);

	//Field
	fieldMatrix = new FieldMatrix();

	//Time Thread
	increaseDiscrepancyX = new TimeThread(100);
	increaseDiscrepancyY = new TimeThread(100);
	timeThreadUpdate1 = new TimeThread(100);
	frontAngleRate = new TimeThread(100);
	movement = new TimeThread(10);

	debug1 = new TimeThread(100);

	timeThreadUpdate1 = new TimeThread(100);
	//Camera
	camera = new Camera("Camera", robotNumber);

	//Solenoid
	solenoid = new Solenoid("Solenoid", robotNumber);

	//Adversaries
	for (int i = 0; i < 25; i++){ adversaryXY[i] = 0; }

	if (robotNumber < 2){
		initializeSimulation(clientID);
	}
	kicked=true;
}
#endif


Robot::~Robot(){

}

//---------- OLD ----------//
//updateRobotX();
//updateRobotY();
//updateBallPossession();
//sonarCleaning();

float frontAngleMov = 0;
float frontMov = 90;

void Robot::execute(){
#if !defined(ARDUINO)
	if (DEBUG && robotNumber<DEBUGROBOTS){
		printf("---------------ROBOT %d executing---------------\n", robotNumber);
		/*printf("[%d][%d] (%0.2f)\n", GetRobotX(), GetRobotY(), compassValue);
		
		robotWithBall == true ? printf("with ball\n") : printf("without ball\n");
		if (debug1->executeThread()){// 100ms

		}
		cout << endl;*/
	}

	//---------- UPDATE ----------//
	updateCompass();//              X Robot / Y Robot / Robot Angle(Gyroscope / Accelerometer / Magnometer)
	updateLightSensor();//        Field Line / Ball possession
	updateRobotAngle();
	//readBluetooth();//            Reveive data
	if (timeThreadUpdate1->executeThread() == true){
		updateSonar();//                XY points / Adjust Robot XY
		updateServo();//              Servo Angle
		updateAdversary();
		updateCamera();//				BallXY(CHANGE!) / GoalProb
	}

	//----- Matrix -----//
	isRobotXKnowed = true;
	isRobotYKnowed = true;
	drawSonarPoints();//            Update points in the field matrix
	//processMatrix();//            Process field matrix (clean, adjust)

	//updateRobotX();
	//updateRobotY();

	//cout << "GoalProb:" << goalProbability() << " ratio:" << realGoalHeight / goalHeightCam << " distNear:" << nearestAdversary;

	//---------- GAME ----------//
	if (robotNumber != 1){
		readBluetooth();//read and update values
		writeBluetooth();
	}

	//move circle
	/*maxPower = 200;

	frontAngleMov >= 360 ? frontAngleMov -= 360 : (frontAngleMov < 0 ? frontAngleMov += 360 : frontAngleMov);
	frontMov >= 360 ? frontMov -= 360 : (frontMov < 0 ? frontMov += 360 : frontMov);

	cout << "Move:" << endl << "   Front Angle:	 " << frontAngleMov << endl << "   Movement Angle: " << frontMov << endl;
	simpleMove(frontAngleMov, frontMov);

	movement->setTimeMilliseconds(10);
	if (movement->executeThread() == true)
	{
		int lostCycles = (movement->lastDiscrepancy) / 100;
		frontAngleMov = frontAngleMov - 0.5*(1 + lostCycles);
		frontMov = frontMov + 1 * (1 + lostCycles);
	}*/
	//deviation
	/* ABLE THIS TO TEST OBSTACLE DEVIATION
	float goalAngle = angleTwoPoints(robotX, robotY, 60, 120);
	if (robotX + robotY != 0){
		circleAvoidObstacle(pathRobot, robotX, robotY, 60, 120, 17, goalAngle);
		vectorMove(pathRobot, 0);
	}*/

	cout << "Estimated ball position:	(" << std::setw(6) << std::left << ballX << "," << std::setw(6) << ballY << ")" << endl;

	if (myPassState != 0 || friendPassState != 0){
		ballPass();
		readBluetooth();//read and update values
		writeBluetooth();
		if (myPassState == 3 && friendPassState == 3){
			myPassState = 0;
			friendPassState = 0;
			if (robotMode == ATTACKER)
				robotMode = DEFENDER;
			else
				robotMode = ATTACKER;
		}
	}
	else if (robotMode == ATTACKER){
		cout << "robot " << robotNumber << " - attacker";
		if (DEBUG && robotNumber<DEBUGROBOTS)
		cout << "Attacker	FrontAngle:" << frontAngle << endl;
		if (isBallKnowed || robotWithBall){//------------------------------------------------------the team know the ball
			if (teamWithBall){//--------------------------------------------------team with the ball
				if (robotWithBall){//---------------------------------------------this robot is with the ball
					if (DEBUG && robotNumber<DEBUGROBOTS)
					cout << "Robot with ball, try goal" << endl;
					//go to kick area and calcule the probability to do goal
				//generatePathArcThreePoints(pathRobot, 40, 163, 91, 130, 140, 163, 10);
					//generatePathArcThreePoints(pathRobot, 140, 163, 91, 130, 40, 163, 10);
					// kickToGoal();
					if (pathRobot.size() > 1){
						if (DEBUG && robotNumber<DEBUGROBOTS)
							cout << "Path with something" << endl;
						//vectorMove(pathRobot, frontAngle);
					}
					else{
						if (DEBUG && robotNumber<DEBUGROBOTS)
							cout << "Path empty"<<endl;

					}
					//simpleMove();
				}
				else
				{
					//ask pass or trade mode
					//catchTheBall();
				}
			}//-------------------------------------------------------------------team without ball
			else{//---------------------------------------------------------------know the ball but no one is with the ball
				//go to ball (avoid objects) and catch the ball
				if (DEBUG && robotNumber<DEBUGROBOTS)
				cout << "Ball Knowed, try to catch" << endl;
				//generateBallPath(pathRobot);
				//vectorMove(pathRobot, frontAngle);
			}
		}//-----------------------------------------------------------------------don't know the ball
		else{
			if (DEBUG && robotNumber<DEBUGROBOTS)
			cout << "Ball Unknowed" << endl;
			//go to defense
			//goToPosition(90,80,frontAngle);
			//goToDefense();
		}
	}
	else if (robotMode == DEFENDER){
		cout << "robot " << robotNumber << "!";
		if (DEBUG && robotNumber<DEBUGROBOTS)
		cout << "Defender	FrontAngle:" <<frontAngle<< endl;
		if (isBallKnowed){//------------------------------------------------------the team know the ball
			if (teamWithBall){//--------------------------------------------------team with the ball
				if (robotWithBall){//---------------------------------------------this robot is with the ball
					if (DEBUG && robotNumber<DEBUGROBOTS)
					cout << "Robot with ball, pass" << endl;
					
					//robotMode = ATTACKER;
					//kickToGoal();
					//simpleMove();
				}
				else
				{
					blockGoal();
				}
			}
			else{
				if (DEBUG && robotNumber<DEBUGROBOTS)
				cout << "Ball Knowed, defense goal-FA: " << frontAngle << endl;
				blockGoal();
			}
		}
		else{
			if (DEBUG && robotNumber<DEBUGROBOTS)
			cout << "Ball Unknowed, defense goal-FA: "<<frontAngle << endl;
			blockGoal();
		}
		
		//simpleMove(frontAngle);
	}
	if (DEBUG && robotNumber<DEBUGROBOTS){
		isBallKnowed == true ? printf("\nBall: (%d,%d)\n", ballX, ballY) : printf("don't know ball\n");
		cout << "RobotWithBall: " << robotWithBall << endl;

		if (debug1->executeThread()){// 100ms

			//cout <<"Compass:	" <<compass->value() << "  " << endl;
		}

		cout << endl;
		//printf("---------------ROBOT %d finishing--------------\n", robotNumber);
	}
#endif
}
// if (DEBUG && robotNumber<DEBUGROBOTS)

//---------- Game ATTACKER----------//
bool Robot::catchTheBall(){
	double ballX_ = ballX;//ballX int to double
	double ballY_ = ballY;//ballY int to double
	if (isBallKnowed){
		float angleRobotBall = angleTwoPoints(robotX, robotY, ballX_, ballY_);//angle robot-ball

		if ((distanceTwoPoints(robotX, robotY, ballX_, ballY_) <= 30) && (robotWithBall == false))
		{
			float distance = distanceTwoPoints(robotX, robotY, ballX_, ballY_);
			maxPower = 100 + 155 * ((distance - 11) / (30 - 11));
#if !defined(ARDUINO)
			cout << "distance:" << distance;
#endif
		}
		else
			maxPower = 255;

		goToPosition(ballX_, ballY_, angleRobotBall);
		if (robotWithBall)
			return true;
		else
			return false;
	}
	else
		return false;

}
bool Robot::kickToGoal(){
	float angleToCenterGoal = angleTwoPoints(robotX, robotY, 91, 243 - 30);

	double ratioValues = realGoalHeight / goalHeightCam;//cam*ratio=cm   cm/ratio=cam
	int FieldOfViewHorizontal = camera->getFieldOfViewH();

	float angleCompass = -compassValue;

	float minorAngle=0;
	float angleToGoal = angleCompass + (float(camera->getFieldOfViewH()) / 2)*(0.5 - goalXCam);

	if (isGoalKnowed)
		frontAngle = angleToGoal;
	else
		frontAngle = angleToCenterGoal;

	float averageXAdversaries = robotX;
	for (int i = 1; i <= numAdversaries; i++){ averageXAdversaries += adversaryXY[(i - 1) * 2 + 1]; }
	averageXAdversaries /= numAdversaries;
	float dangerSize = averageXAdversaries - robotX;//if there is more objects on the left(+) of the robot, is danger kick to the left, so, kick to the right(-)
	//if (dangerSize>=0)
		//minorAngle = -compassValue + (float(camera->getFieldOfViewH()) / 2)*(0.5 - goalXCam + (goalLengthCam / 2) - (8 / ratioValues));
	//else
	minorAngle = angleCompass + (float(camera->getFieldOfViewH()) / 2)*(0.5 - goalXCam /*-(goalLengthCam / 2) + (8 / ratioValues)*/);

	angleCompass < 0 ? angleCompass += 360 : angleCompass;
	angleCompass >= 360 ? angleCompass -= 360 : angleCompass;
	angleToGoal < 0 ? angleToGoal += 360 : angleToGoal;
	angleToGoal >= 360 ? angleToGoal -= 360 : angleToGoal;
	minorAngle < 0 ? minorAngle += 360 : minorAngle;
	minorAngle >= 360 ? minorAngle -= 360 : minorAngle;

	//cout << "cmp:" << -compassValue << " Prob:" << goalProbability() << " Xcam:" << goalXCam << " aToGoal:" << angleToGoal << endl;
	//cout << "mAng:" << minorAngle << "(" << simpleMove(minorAngle) << ")"
	//	<< " cmp:" << angleCompass << " prop:" << goalProbability() << " advX:" << averageXAdversaries - robotX << "goalL:" << realGoalLength << endl;
	//simpleMove();
	if (isGoalKnowed){
		if (DEBUG && robotNumber < DEBUGROBOTS)
			cout << "	goalKnowed" << endl;

		static bool firstMov = true;//save the last moviment

		/*if (goalProbability() >= 0.5){
			if (DEBUG && robotNumber < DEBUGROBOTS)
				cout << "Kick!" << endl;
			kick();
			extApi_sleepMs(500);
			return true;
		}*/

		if (robotY < 130)
		{
			firstMov = false;
			if (goalXCam >= 0.5){
				if (DEBUG && robotNumber < DEBUGROBOTS)
					cout << "		AttackGoalDir" << endl;
				//extApi_sleepMs(2000);
				generatePathLine(pathRobot, robotX, robotY, 40, 163, 5);
			}
			if (goalXCam < 0.5){
				if (DEBUG && robotNumber < DEBUGROBOTS)
					cout << "		AttackGoalEsq" << endl;
				//extApi_sleepMs(2000);
				generatePathLine(pathRobot, robotX, robotY, 140, 163, 5);

			}
		}
		else if (pathRobot.size() == 0)
		{
			if (DEBUG && robotNumber < DEBUGROBOTS)
				cout << "	pass the ball" << endl;
			ballPass();
			/*if (robotX < 90){
				if (DEBUG && robotNumber < DEBUGROBOTS)
					cout << "		AttackGoalDir->Esq" << endl;
				generatePathArcThreePoints(pathRobot, 40, 163, 91, 130, 140, 163, 10);
			}
			if (robotX >= 90){
				if (DEBUG && robotNumber < DEBUGROBOTS)
					cout << "		AttackGoalEsq->Dir" << endl;
				generatePathArcThreePoints(pathRobot, 140, 163, 91, 130, 40, 163, 10);
			}*/
			return false;
		}
		cout << "	nothing		rY:" << robotY << " Path:" << pathRobot.size() << endl;
		return false;
	}
	else{
			cout << "don't know goal, go to center" << angleToCenterGoal << endl;
			goToPosition(91,140,0);
			cout << "cG:" << angleToCenterGoal << "  a" << angleCompass<<endl;
			simpleMove(angleToCenterGoal);
		return false;
	}
	return false;
}

bool Robot::ballPass(){
	/*	myPassState && fiendPassState
		1 - will pass the ball to the other
		2 - aligned to pass
		3 - kicked the ball/received the ball
	*/
	if (DEBUG && robotNumber < DEBUGROBOTS)
		cout << "PassStateFriend:" << (int)friendPassState << " MyPassBall: " << (int)myPassState << endl;

	if (myPassState==0 && robotWithBall)
		myPassState = 1;

	float angleFriend = angleTwoPoints(robotX, robotY, friendXY[0], friendXY[1]);//to align
	static bool tryCatchBall = false;

	if (simpleMove(angleFriend)==true){
		myPassState = 2;
	}

	if (myPassState == 2 && friendPassState >= 2){
		if (robotWithBall == true){
			kick();
			myPassState = 3;
		}
		else{
			catchTheBall();
			tryCatchBall = true;
		}
	}
	if (tryCatchBall){
		if (robotWithBall == false)
			catchTheBall();
		else
			myPassState = 3;
		return true;
	}
	return false;
}

float Robot::goalProbability(){
	float probability = 1;
	double ratioValues = realGoalLength / goalLengthCam;

	//----- if is possible occur a goal -----//
	float probMaster = 0;
	if (robotWithBall && isGoalKnowed)
		probMaster = 1;
	//----- if is possible kick and the ball enter the goal -----//
	float probBallEnterGoal = 1;

	float goalToRightCam = goalXCam + (goalLengthCam / 2) - 0.5;
	float goalToLeftCam = -(goalToRightCam - (goalLengthCam));
	float goalToRight = (goalToRightCam)*ratioValues;
	float goalToLeft = (goalToLeftCam)*ratioValues;
	//cout << "\nGOALR:" << goalToRightCam << " " << goalToRight;
	//cout << " GOALL:" << goalToLeftCam << " " << goalToLeft << endl;
	if (goalToRight < 4 || goalToLeft < 4)//if the ball can enter the goal
		probBallEnterGoal = 0;

	//----- distance robot to goal -----//
	float probDistGoal = 1;
	float distRobotGoal = distanceTwoPoints(robotX, robotY, 90, 213);
	float minDistGoal = 60;// less than this distance return 1
	float maxDistGoal = 130;// bigger than this distance return 0
	probDistGoal = 1 - ((distRobotGoal - minDistGoal) / (maxDistGoal - minDistGoal));
	//output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
	distRobotGoal < minDistGoal ? probDistGoal = 1 : probDistGoal;
	distRobotGoal > maxDistGoal ? probDistGoal = 0 : probDistGoal;

	//----- goal length prob -----//
	float probLenGoal = realGoalLength / 30;
	probLenGoal > 1 ? probLenGoal = 1 : probLenGoal;

	//----- prob adversary don't defende the goal -----//
	/*float probAdversary = 1;
	float nearestAdversary = 1000;//nearest and with Y bigger than robotY(in the front of the robot)
	for (int i = 1; i <= numAdversaries; i++){
		int advX = adversaryXY[(i - 1) * 2 + 1];
		int advY = adversaryXY[(i - 1) * 2 + 2];
		int dist = distanceTwoPoints(sonarPointX[0], 213, advX, advY);
		if (dist < nearestAdversary && advY>robotY)
			nearestAdversary = dist;
	}
	probAdversary = nearestAdversary / 90;
	probAdversary > 1 ? probAdversary = 1 : probAdversary;*/

	probability *= probMaster*probBallEnterGoal*probLenGoal*probDistGoal/*probAdversary*/;

	//cout << "Prob goal:" << probability << "probMaster:" << probMaster << " probEnter:" << probBallEnterGoal << endl << " probLen:" << probLenGoal
	//	<< " probDist:" << probDistGoal; //<< " probAdv:" << probAdversary << endl;

	return probability;//need to be calculate
}
//---------- Game DEFENDER----------//
bool Robot::blockGoal(){
	maxPower = 255;
	double ballX_ = ballX;//ballX int to double
	double ballY_ = ballY;//ballY int to double

	static double posX;
	static double posY;
	static float lastPosX = 0;

	if (isBallKnowed){
		//float angleRobotBall = angleTwoPoints(robotX, robotY, ballX_, ballY_);//angle robot-ball

		//line equation --> f(x)=ax+b -->a=(Yball-Ygoal)/(Xball-Xgoal)
		//							  -->b= substitute "a" in equation f(x)=ax+b

		/*if (ballX_ == 61 || ballX_ == 121 || ballX_ == 0){
			ballX_++;
			cout << ballX_;
		}
		//----- line equation ball-leftLock  -----//
		double aLeft = (ballY_ - 30) / (ballX_ - 61);
		aLeft != 0 ? aLeft : 1;//						"a" value can't be 0
		double bLeft = ballY_ - (aLeft*ballX_);
		//----- line equation ball-rightLock  -----//
		double aRight = (ballY_ - 30) / (ballX_ - 121);
		aRight != 0 ? aRight : 1;//						"a" value can't be 0
		double bRight = ballY_ - (aRight*ballX_);
		//----- robot coordenates to protect  -----// substitute in eqution(Y=45) --> 45 = ax+b --> (45-b)/a = x
		double xLeft = (45 - bLeft) / aLeft;//
		double xRight = (45 - bRight) / aRight;
		double posX = (xLeft + xRight) / 2;//the robot will stay in the middle*/

		posX = ballX_;

		posX <= 61 ? posX = 66 : posX;
		posX >= 121 ? posX = 116 : posX;

		goToPosition(posX, 45, 0);
		lastPosX = posX;
		return true;
	}
	else{
		goToPosition(lastPosX, 45, 0);
		return false;
	}
}

bool Robot::goToDefense(){
	static bool lastPosRight = true;
	maxPower = 255;

	if (lastPosRight == true){
		if (goToPosition(30, 70, frontAngle)){
			lastPosRight = false;
		}
	}
	else{
		if (goToPosition(150, 70, frontAngle)){
			lastPosRight = true;
		}
	}	
	return true;
}
//---------- Moviment ----------//
bool Robot::simpleMove(float frontRobotAngle, float moveAngle){
	if (DEBUGMOV && robotNumber<DEBUGROBOTS)
	cout << "SimpleMove: ";
	static int lastFrontRobotAngle;
	static int lastMoveAngle;
	
	updateCompass();

	float PID = abs(frontPID(360 - frontRobotAngle));
	//cout << "PID:" << PID << endl;

	float compassRobot = (360 - compassValue) - frontRobotAngle;//the 0 is the frontRobotAngle when the robot move
	while (compassRobot < 0)
		compassRobot += 360;
	while (compassRobot >= 360)
		compassRobot -= 360;
	if (frontRobotAngle == 1000 && moveAngle == 1000)
	{
		if (DEBUGMOV && robotNumber<DEBUGROBOTS)
		cout << "	robot stopped" << endl;
		relativeMotor(FR, 0);
		relativeMotor(FL, 0);
		relativeMotor(BR, 0);
		relativeMotor(BL, 0);
	}
	else if (moveAngle != 1000 && (compassRobot<45 || compassRobot>315))//--------------------------------------------------------(normal)
	{
		if (DEBUGMOV && robotNumber<DEBUGROBOTS)
		cout << "	normal" << endl;
		while (moveAngle >= 360)//more than 360 angle tratament
			moveAngle -= 360;
		while (moveAngle < 0)//negative angle tratament
			moveAngle += 360;

		while (frontRobotAngle >= 360)//more than 360 angle tratament
			frontRobotAngle = frontRobotAngle - 360;
		while (frontRobotAngle < 0)//negative angle tratament
			frontRobotAngle = frontRobotAngle + 360;

		bool distance90 = isNear(moveAngle, frontRobotAngle, 90);//true=is near

		float compass180 = compassValue;// 0 -> 45 -> 90 -> 135 -> 180 -> -135 -> -90 -> -45-> 0

		if (compass180 > 180)//left negative - right positive
		{
			compass180 = (360 - compass180)*(-1);
		}

		float robotAngle180 = moveAngle;
		if (robotAngle180 > 180)
		{
			robotAngle180 = (360 - robotAngle180)*(-1);
		}

		float alpha = compass180;//left negative | right positive		(0 degree to robot front angle now)
		float beta = robotAngle180;//left positive | right negative		(0 degree to angle moviment)

		//-----------------------------------------spin calcule
		int spinTimes = 0;//Number of spins

		if (compassValue > 180)
			spinTimes = 4;

		int spinTime_auxiliary = 0;
		int RealNumberMotor;//The real number after degree tratament

		if (compassValue <= 180)
		{
			while (compassValue - 45 - (90 * spinTime_auxiliary) >= 0)
			{
				spinTimes++;
				spinTime_auxiliary++;
			}
		}
		else
		{
			while ((360 - compassValue) - 45 - (90 * spinTime_auxiliary) >= 0)
			{
				spinTimes--;
				spinTime_auxiliary++;
			}
		}

		if (spinTimes < 0)
			spinTimes = 4 + spinTimes;

		//-----------------------------------------spin calcule

		float angle = 45 + alpha + beta - spinTimes * 90;//the robot always will move in the robotAngle angle

		while (angle < 0)
			angle = angle + 360;
		while (angle >= 360)
			angle = angle - 360;

		//cout << angle << "= 45+(" << alpha << ")+(" << beta << ")-(" << spinTimes * 90<<")|";
		//cout << angle << "|";
		angle = angle*M_PI / 180;//transform moviment angle trigonometric circle(radians)

		double diagonalLeft = sin(angle);
		double diagonalRight = cos(angle);

		if (sin(angle) < 0)//value tratament: positive
			diagonalLeft = diagonalLeft*(-1);
		if (cos(angle) < 0)
			diagonalRight = diagonalRight*(-1);

		float correction = 1;
		//value tratament: if diagonal>1, diagonal=1 (the other decreases proportion)
		if (diagonalRight > diagonalLeft)
			correction = 1 / diagonalRight;
		else
			correction = 1 / diagonalLeft;

		if (sin(angle) < 0)//value tratament: positive
			diagonalLeft = diagonalLeft*(-1);
		if (cos(angle) < 0)
			diagonalRight = diagonalRight*(-1);

		diagonalRight = diagonalRight * correction * 255 * (static_cast<float>(maxPower) / 255);//float power
		diagonalLeft = diagonalLeft * correction * 255 * (static_cast<float>(maxPower) / 255);//float power

		if (compassRobot > 180)
		{
			if (diagonalLeft <= 0)
			{
				relativeMotor(FR, diagonalLeft*PID);
				relativeMotor(BL, diagonalLeft*(1 / PID));
			}
			else
			{
				relativeMotor(FR, diagonalLeft*(1 / PID));
				relativeMotor(BL, diagonalLeft*PID);
			}
			if (diagonalRight <= 0)
			{
				relativeMotor(FL, diagonalRight*(1 / PID));
				relativeMotor(BR, diagonalRight*PID);
			}
			else
			{
				relativeMotor(FL, diagonalRight*PID);
				relativeMotor(BR, diagonalRight*(1 / PID));
			}
		}
		else if (compassRobot <= 180)
		{
			if (diagonalLeft <= 0)
			{
				relativeMotor(FR, diagonalLeft*(1 / PID));
				relativeMotor(BL, diagonalLeft*PID);
			}
			else
			{
				relativeMotor(FR, diagonalLeft*PID);
				relativeMotor(BL, diagonalLeft*(1 / PID));
			}
			if (diagonalRight <= 0)
			{
				relativeMotor(FL, diagonalRight*PID);
				relativeMotor(BR, diagonalRight*(1 / PID));
			}
			else
			{
				relativeMotor(FL, diagonalRight*(1 / PID));
				relativeMotor(BR, diagonalRight*PID);
			}
		}
	}
	else//-------------------------------------------------------------------(spin)
	{
		PID = 1 - PID;
		if (DEBUGMOV && robotNumber<DEBUGROBOTS)
		cout << "	Spin PID:"<<PID;
		
			if (compassRobot <= 0.5)//Stop
			{
				if (DEBUGMOV && robotNumber<DEBUGROBOTS)
				cout << "		(stop)" << endl;
				relativeMotor(FR, 0);
				relativeMotor(BL, 0);

				relativeMotor(FL, 0);
				relativeMotor(BR, 0);
			}
			else if (compassRobot <= 180)//Slow motion correction 
			{
				if (DEBUGMOV && robotNumber<DEBUGROBOTS)
					cout << "		(Slow motion right)" << "left:" << maxPower*PID << "right:" << maxPower*PID*(-1) << endl;
				relativeMotor(FR, maxPower*PID*(-1));
				relativeMotor(BL, maxPower*PID);

				relativeMotor(FL, maxPower*PID);
				relativeMotor(BR, maxPower*PID*(-1));
			}
			/*else if (compassRobot < 180)//Fast motion correction
			{
				if (DEBUGSM)
				cout << "(Fast motion right)" << endl;
				relativeMotor(FR, maxPower*(1 / PID)*(-1));
				relativeMotor(BL, maxPower*(1 / PID));

				relativeMotor(FL, maxPower*(1 / PID));
				relativeMotor(BR, maxPower*(1 / PID)*(-1));

			}*/
			/*if (compassRobot >= 359)//Stop
			{
				if (DEBUGSM)
				cout << "(Stop)" << endl;
				relativeMotor(FR, 0);
				relativeMotor(BL, 0);

				relativeMotor(FL, 0);
				relativeMotor(BR, 0);
			}*/
			else if (compassRobot > 180)//Slow motion correction 
			{
				if (DEBUGMOV && robotNumber<DEBUGROBOTS)
					cout << "		(Slow motion left)" << "left:" << maxPower*PID*(-1) << "right:" << maxPower*PID << endl;
				relativeMotor(FR, maxPower*PID);
				relativeMotor(BL, maxPower*PID*(-1));

				relativeMotor(FL, maxPower*PID*(-1));
				relativeMotor(BR, maxPower*PID);

			}
			/*else if (compassRobot >= 180)//Fast motion correction 
			{
				if (DEBUGSM)
				cout << "(Fast motion left)" << endl;
				relativeMotor(FR, maxPower*(1 / PID));
				relativeMotor(BL, maxPower*(1 / PID)*(-1));

				relativeMotor(FL, maxPower*(1 / PID)*(-1));
				relativeMotor(BR, maxPower*(1 / PID));

			}*/
		//}
	}
	//if (robot)

	compassValue = 360 - compassValue;

	lastFrontRobotAngle = frontRobotAngle;
	lastMoveAngle = moveAngle;
	//cout << endl << "front:" << frontRobotAngle << "cmps:" << compassValue << endl;
	return isNearAngle(compassValue, frontRobotAngle, 2);
}
bool Robot::goToPosition(int X, int Y, float frontRobotAngle, bool smooth){//smooth is really needed?
	X < 30  ? X = 30 : X;
	X > 152 ? X=151 : X;
	Y < 30 + 11 ? Y=41 : Y;
	Y > 213 - 11 ? Y = 201 : Y;
	if (DEBUG && robotNumber<DEBUGROBOTS)
	cout << "goToPosition- X:" << X << " Y:" << Y << endl;
	if (smooth){
		if (distanceTwoPoints(robotX, robotY, X, Y) <= 40)
		{
			int distance = distanceTwoPoints(robotX, robotY, X, Y);
			maxPower = 50+ 205*(distance / 40);
		}
		else
			maxPower = 255;
	}


	if (!(isNear(robotX, X, 3) && isNear(robotY, Y, 3))){
		int movimentAngle = angleTwoPoints(robotX, robotY, X, Y);
		simpleMove(frontRobotAngle, movimentAngle);
		return false;
	}
	else
	{
		simpleMove(frontRobotAngle);//stop
		return true;
	}

}
bool Robot::vectorMove(vector<int>& path, float frontRobotAngle){
	if (path.size() >= 2){
		int nextX = path[0];
		int nextY = path[1];
		if (DEBUGMOV && robotNumber<DEBUGROBOTS){
			cout << "nextPath:(" << nextX << "," << nextY << ")" << endl;
		}
		while (isNear(robotX, nextX, 10) && isNear(robotY, nextY, 10)){
			if (path.size() >= 2){
				path.erase(path.begin());
				path.erase(path.begin());
				if (path.size() >= 2){
					nextX = path[0];
					nextY = path[1];
				}
				else{
					nextX = 0;
					nextY = 0;
					return true;
				}
			}
			else{
				assert(" vectorMove: path too low to erase");
			}
		}
		if (distanceTwoPoints(nextX, nextY, ballX, ballY) < 7){
			catchTheBall();
		}
		else{
			goToPosition(nextX, nextY, frontRobotAngle);
		}
		

		return true;
	}
	else{
		if (DEBUGMOV && robotNumber<DEBUGROBOTS){
			cout << "nextPath:()" << endl;
		}
		return false;
	}
}

int lastSense=0;
int distancePath1 = 0;
int distancePath2 = 0;
void Robot::circleAvoidObstacle(vector<int>& pathPoints, int Xcircle, int Ycircle, int goalX, int goalY, int radius, int oppositeAngle, int sense, int recursion)
{
	//cout << "finalDistancePath1: " << distancePath1 << "	finalDistancePath2: " << distancePath2 << endl;

	int firstOppositeAngle = oppositeAngle;
	//sense=0	clockwise and anti-clockwise
	//sense=1	clockwise
	//sense=2	anti-clockwise
	bool clockwise = true;
	int pointsFounded = 2;

	int lastPointPath = -1;//store pathPoints[i] "size value not 0"
	/*for (int i = 0; pathPoints[i] > 0; i++)
	{
		lastPointPath = i;
	}*/

	if (sense==0)
		pathPoints.clear();

	lastPointPath = pathPoints.size() - 1;
	
	pathPoints.push_back(Xcircle);//create a new X path
	pathPoints.push_back(Ycircle);//create a new X path

	int circleX1 = 0;
	int circleY1 = 0;
	int circleX2 = 0;
	int circleY2 = 0;

	if (sense == 2)//sense defines to where the path will continue to be create (clock/clockwise)
		clockwise = false;
	if (sense != 0)
		pointsFounded--;//if is not the first circle, only one pointsFounded

	int contWhile = 0;

	while ((clockwise == true && pointsFounded > 0) && (contWhile < 80))
	{
		float trigonometricAngle = (oppositeAngle + 90)*M_PI / 180;//angle in radians
		circleX2 = Xcircle + int(radius*cos(trigonometricAngle));
		circleY2 = Ycircle + int(radius*sin(trigonometricAngle));
		if (fieldMatrix->fillCircleRead(circleX2, circleY2, radius, 1) == 0)
		{
			//drawCircle(circleX2, circleY2, radius, 5);
			pointsFounded--;
			if (pointsFounded == 1 && sense == 0)
			{
				clockwise = false;
				oppositeAngle = firstOppositeAngle;
			}
		}
		//fieldMatrix->drawFillCircle(circleX2, circleY2, 3, 2);
		oppositeAngle -= 5;
		contWhile++;
	}
	contWhile = 0;
	while ((clockwise == false && pointsFounded > 0) && (contWhile < 80))
	{
		float trigonometricAngle = (oppositeAngle + 90)*M_PI / 180;//angle in radians
		circleX1 = Xcircle + int(radius*cos(trigonometricAngle));
		circleY1 = Ycircle + int(radius*sin(trigonometricAngle));
		if (fieldMatrix->fillCircleRead(circleX1, circleY1, radius, 1) == 0)
		{
			//drawCircle(circleX1, circleY1, radius, 5);
			pointsFounded--;
		}
		//drawFillCircle(circleX1, circleY1, 3, 5);
		oppositeAngle += 5;
		contWhile++;
	}
	//-----see circles-----//
	if (distanceTwoPoints(circleX1, circleY1, goalX, goalY) < distanceTwoPoints(circleX2, circleY2, goalX, goalY))
	{
		//drawCircle(circleX1, circleY1, radius, 2);
	}
	else
	{
		//drawCircle(circleX2, circleY2, radius, 2);
	}

	//----------//
	if (recursion == 20)
	{
		//find final distance in path 2
		oppositeAngle = angleTwoPoints(circleX1, circleY1, goalX, goalY);
		circleAvoidObstacle(pathPoints, circleX1, circleY1, goalX, goalY, radius, oppositeAngle, 2, recursion - 1);
		distancePath2 = distanceTwoPoints(goalX, goalY, pathPoints[pathPoints.size() - 2], pathPoints[pathPoints.size() - 1]);
		pathPoints.clear();
		//find final distance in path 1
		oppositeAngle = angleTwoPoints(circleX2, circleY2, goalX, goalY);
		circleAvoidObstacle(pathPoints, circleX2, circleY2, goalX, goalY, radius, oppositeAngle, 1, recursion - 1);
		distancePath1 = distanceTwoPoints(goalX, goalY, pathPoints[pathPoints.size() - 2], pathPoints[pathPoints.size() - 1]);
		pathPoints.clear();

			if (distancePath1 < distancePath2){
				oppositeAngle = angleTwoPoints(circleX2, circleY2, goalX, goalY);
				circleAvoidObstacle(pathPoints, circleX2, circleY2, goalX, goalY, radius, oppositeAngle, 1, recursion - 1);
			}
			else{
				oppositeAngle = angleTwoPoints(circleX1, circleY1, goalX, goalY);
				circleAvoidObstacle(pathPoints, circleX1, circleY1, goalX, goalY, radius, oppositeAngle, 2, recursion - 1);
			}
			/*}
		else{
			if ((distanceTwoPoints(circleX1, circleY1, goalX, goalY) <= distanceTwoPoints(circleX2, circleY2, goalX, goalY))
				&& fieldMatrix->bigLineRead(Xcircle, Ycircle, goalX, goalY, 11, 1) != 0)
			{
				if (lastSense == 0)
					lastSense = 2;

				//cout << "clock";
				oppositeAngle = angleTwoPoints(circleX1, circleY1, goalX, goalY);
				//drawCircle(circleX1, circleY1, radius, 2);
				circleAvoidObstacle(pathPoints, circleX1, circleY1, goalX, goalY, radius, oppositeAngle, 2, recursion - 1);

				distancePath2 = distanceTwoPoints(goalX, goalY, pathPoints[pathPoints.size() - 2], pathPoints[pathPoints.size() - 1]);

			}
			else if ((distanceTwoPoints(circleX2, circleY2, goalX, goalY) <= distanceTwoPoints(circleX1, circleY1, goalX, goalY)) &&
				(fieldMatrix->bigLineRead(Xcircle, Ycircle, goalX, goalY, 11, 1) != 0))
			{
				if (lastSense == 0)
					lastSense = 1;
				//cout << "antclock";
				oppositeAngle = angleTwoPoints(circleX2, circleY2, goalX, goalY);
				//drawCircle(circleX2, circleY2, radius, 2);
				circleAvoidObstacle(pathPoints, circleX2, circleY2, goalX, goalY, radius, oppositeAngle, 1, recursion - 1);

				distancePath1 = distanceTwoPoints(goalX, goalY, pathPoints[pathPoints.size() - 2], pathPoints[pathPoints.size() - 1]);
			}
		}*/
	}
	else if (recursion > 0)
	{
		//cout << "bat";
		if ((fieldMatrix->bigLineRead(Xcircle, Ycircle, goalX, goalY, 11, 1) != 0) && sense == 2)
		{
			oppositeAngle = angleTwoPoints(circleX1, circleY1, goalX, goalY);
			//drawCircle(circleX1, circleY1, radius, 2);
			circleAvoidObstacle(pathPoints, circleX1, circleY1, goalX, goalY, radius, oppositeAngle, 2, recursion - 1);
		}
		else if ((fieldMatrix->bigLineRead(Xcircle, Ycircle, goalX, goalY, 11, 1) != 0) && sense == 1)
		{
			oppositeAngle = angleTwoPoints(circleX2, circleY2, goalX, goalY);
			//drawCircle(circleX2, circleY2, radius, 2);
			circleAvoidObstacle(pathPoints,circleX2, circleY2, goalX, goalY, radius, oppositeAngle, 1, recursion - 1);
		}
	}
}
//---------- Path ----------//
bool Robot::generateBallPath(vector<int>& path){
	static int _ballX = 0;
	static int _ballY = 0;//better: searchChange()
	static int _robotX = 0;
	static int _robotY = 0;
	float ballAngle = 0;
	float atualAngle = 0;

	//generate path next to the ball
	ballAngle = angleTwoPoints(robotX, robotY, ballX, ballY);

			
			if (distanceTwoPoints(robotX, robotY, ballX, ballY) > 50){
				generatePathLine(path, robotX, robotY, ballX, ballY, 5);
				if (DEBUG && robotNumber<DEBUGROBOTS)
				cout << "GOING TO BALL" << endl;
			}
			else{
				if (DEBUG && robotNumber<DEBUGROBOTS)
				cout << "CATCH BALL" << endl;
				catchTheBall();
			}
			//path.push_back(ballX);//cos*a -- a=distance from the ball equal
			//path.push_back(ballY);//sin*a -- a=distance from the ball equals
	return true;
}	
bool Robot::generatePathLine(vector<int>& path, int x0, int y0, int x1, int y1,int scale){
	path.clear();

	float totalHip = distanceTwoPoints(x0, y0, x1, y1);
	float Xdistance = x1-x0;
	float Ydistance = y1-y0;

	int totalPoints = totalHip / scale;
	for (int i = 1; i < totalPoints; i++){
		int actualHip = (totalHip / totalPoints)*i;
		float hipRatio = actualHip / totalHip;
		path.push_back(int(x0 + Xdistance*hipRatio));
		path.push_back(int(y0 + Ydistance*hipRatio));
	}

	//
	return true;
}
bool Robot::generatePathArcThreePoints(vector<int>& path, float x1, float y1, float x2, float y2, float x3, float y3, int scale){
	//help from http://www.ucancode.net/faq/C-Line-Intersection-2D-drawing.htm
	static float lastX1, lastY1, lastX2, lastY2, lastX3, lastY3;

	bool execute = ((x1 != lastX1 && y1 != lastY1 && x2 != lastX2 && y2 != lastY2 && x3 != lastX3 && y3 != lastY3) || pathRobot.size()==0);
	if (DEBUG && robotNumber<DEBUGROBOTS)
		cout << "Generate Path" << execute << endl;

	if (execute){
		if (DEBUG && robotNumber<DEBUGROBOTS)
			cout << "GEROU INICIO" << x1<<" "<<y1 << endl;
		path.clear();
		lastX1 = x1; lastY1 = y1;
		lastX2 = x2; lastY2 = y2;
		lastX3 = x3; lastY3 = y3;
		


		float xc;
		float yc;

		//points one and two (slope-intercept) y=ax+b
		float x1m = (x1 + x2) / 2;
		float y1m = (y1 + y2) / 2;
		float A1s = -(x2 - x1) / (y2 - y1);
		float B1s = y1m - A1s*x1m;
		//points one and two (standard form) Ax+By=C
		float A1 = A1s;
		float B1 = -1;
		float C1 = -B1s;

		//points two and three (slope-intercept) y=ax+b
		float x2m = (x3 + x2) / 2;
		float y2m = (y3 + y2) / 2;
		float A2s = -(x2 - x3) / (y2 - y3);
		float B2s = y2m - A2s*x2m;
		//points  two and three (standard form) Ax+By=C
		float A2 = A2s;
		float B2 = -1;
		float C2 = -B2s;

		float det = A1*B2 - A2*B1;
		if (det == 0){
			//Lines are parallel
		}
		else{
			xc = (B2*C1 - B1*C2) / det;
			yc = (A1*C2 - A2*C1) / det;

			float radius = distanceTwoPoints(xc, yc, x1, y1);

			float angle1 = atan2(y1 - yc, x1 - xc) * 180 / M_PI;//atan2(y,x)
			float angle2 = atan2(y2 - yc, x2 - xc) * 180 / M_PI;//atan2(y,x)
			float angle3 = atan2(y3 - yc, x3 - xc) * 180 / M_PI;//atan2(y,x)

			float angleNow = angle1;

			angle1 < 0 ? angle1 + 360 : angle1;
			angle2 < 0 ? angle2 + 360 : angle2;
			angle3 < 0 ? angle3 + 360 : angle3;

			//cout << "angles:" << angle1 << " " << angle2 << " " << angle3 << endl;
			//extApi_sleepMs(10000);

			if (path.size() == 0){
				if (angle1 < angle3){
					while (angleNow < angle3){
						//cout << "now:" << angleNow << endl;
						float pointX = xc + radius*cos(angleNow / 180 * M_PI);
						float pointY = yc + radius*sin(angleNow / 180 * M_PI);
						//cout << angleNow << "<" << angle1 << "pX:" << pointX << "pY:" << pointY << endl;
						path.push_back(int(pointX));
						path.push_back(int(pointY));
						angleNow += scale;
					}
				}
				else{
					while (angleNow > angle3){
						//cout << "now:" << angleNow << endl;
						float pointX = xc + radius*cos(angleNow / 180 * M_PI);
						float pointY = yc + radius*sin(angleNow / 180 * M_PI);
						//cout << angleNow << "<" << angle1 << "pX:" << pointX << "pY:" << pointY << endl;
						path.push_back(int(pointX));
						path.push_back(int(pointY));
						angleNow -= scale;
					}
				}
			}
			return true;
		}
	}
}
//---------- Motor ----------//
void Robot::relativeMotor(MotorsEnum motorName, int motorPower){
	int spinTimes = 0;//Number of motor spins|EX: spin=2 -> Sonar 1 turns 3/Sonar 2 turns 4
	int motorNumber = 0;

	//motorPower>255 ? motorPower = 255 : motorPower;

	switch (motorName)
	{
	case FR:
		motorNumber = 1;
		break;
	case BR:
		motorNumber = 2;
		break;
	case BL:
		motorNumber = 3;
		break;
	case FL:
		motorNumber = 4;
		break;
	}

	if (compass->value() > 180)
		spinTimes = 4;

	int spinTime_auxiliary = 0;

	if (compassValue <= 180)
	{
		while (compassValue - 45 - (90 * spinTime_auxiliary) >= 0)
		{
			spinTimes++;
			spinTime_auxiliary++;
		}
	}
	else
	{
		while ((360 - compassValue) - 45 - (90 * spinTime_auxiliary) >= 0)
		{
			spinTimes--;
			spinTime_auxiliary++;
		}
	}

	if (spinTimes < 0)
		spinTimes = 4 + spinTimes;
	int RealNumberMotor;//The real number after degree tratament

	RealNumberMotor = motorNumber - spinTimes;
	while (RealNumberMotor > 4)
		RealNumberMotor = RealNumberMotor - 4;
	while (RealNumberMotor < 1)
		RealNumberMotor = RealNumberMotor + 4;

	switch (RealNumberMotor)
	{
	case 1:
		if (spinTimes == 2 || spinTimes == 3)
			motorPower *= (-1);
		motors[0]->power(motorPower);
		break;
	case 2:
		if (spinTimes == 1 || spinTimes == 2)
			motorPower *= (-1);
		motors[1]->power(motorPower);
		break;
	case 3:
		if (spinTimes == 2 || spinTimes == 3)
			motorPower *= (-1);
		motors[2]->power(motorPower);
		break;
	case 4:
		if (spinTimes == 1 || spinTimes == 2)
			motorPower *= (-1);
		motors[3]->power(motorPower);
		break;
	}
}
float Robot::frontPID(float frontAngle){
	float error;
	int realAngle180;//Opposite of the front of the moviment

	compassValue = compass->value();

	error = distanceTwoAngles(compassValue, frontAngle);
	/*if (frontAngle < 180)
	{
		realAngle180 = frontAngle + 180;
		if ((compassValue >= frontAngle) && (compassValue < realAngle180))
			error = compassValue - frontAngle;
		else
		{
			if ((compassValue >= 0) && (compassValue < frontAngle))
			{
				error = frontAngle - compassValue;
			}
			else
			{
				error = frontAngle + (360 - compassValue);
			}
		}
	}
	else
	{
		realAngle180 = frontAngle - 180;
		if ((compassValue <= frontAngle) && (compassValue > realAngle180))
		{
			error = frontAngle - compassValue;
		}
		else
		{
			if (compassValue < realAngle180)
				error = (360 - frontAngle) + compassValue;
			else
			{
				error = compassValue - frontAngle;
			}
		}
	}*/
	//error Ok

	if (error < 0.5)
		error = 0;
	else if (error < 1)
		error = 1;
	if (error > 89)
		error = 89;

	if (DEBUGMOV && robotNumber<DEBUGROBOTS)
		cout << "PIDerror:" << error << "(c" << compassValue << "," << frontAngle<<")"<<endl;

	if (isFirstPID == true)
	{
#if defined(ARDUINO)
		lastPIDprocess = millis();
#else
		lastPIDprocess = clock();
#endif 
		isFirstPID = false;
	}

	float PID = 0;
	float currentDegree = compassValue;
#if defined(ARDUINO)
	float deltaTime = (millis() - lastPIDprocess) / 1000.0;
	lastPIDprocess = millis();
#else
	float deltaTime = (clock() - lastPIDprocess) / 1000.0;
	lastPIDprocess = clock();
#endif 
	//P
	P = error * Kp;

	//I
	/*I = I + (error * Ki) * deltaTime;*/

	//D
	float degreesDifference = lastDegree - currentDegree;//if negative -> increasing | positive -> decreasing
	if (degreesDifference > 90)//have something wrong in the calculus (360->0)
	{
		if (degreesDifference < 0)
		{
			degreesDifference = 360 - degreesDifference;//if the angle decrease
		}
		else
		{
			degreesDifference = degreesDifference - 360;//if the angle increase
		}
	}
	D = degreesDifference * Kd / deltaTime;
	lastDegree = currentDegree;

	// Resultado PID
	PID = 1/(P + I + D);

	PID > 1 ? PID = 1 : PID = PID;

	//cout << "PID:" << PID << "|error:" << error << "(P:" << P << ")" << "(I:" << I << ")" << "(D:" << D << ")" << endl;
	return PID;
}
//----- Bluetooth -----//
void  Robot::readBluetooth(){
	//cout << "BLUETOOTHread: ";

	int friendX = 0;
	int friendY = 0;
	bool isBallKnowedFriend = 0;
	int _ballX = 0;
	int _ballY = 0;

	int contWhile=0;

	while (bluetooth->read(robotNumber) != -31 && contWhile < 20){ contWhile++; }contWhile = 0;

	if (bluetooth->read(robotNumber) == -41){
		friendX = bluetooth->read(robotNumber);// cout << (int)friendX << " ";//pi
		friendY = bluetooth->read(robotNumber);// cout << (int)friendY << " ";//pi
		isBallKnowedFriend = bluetooth->read(robotNumber);// cout << (int)isBallKnowedFriend << " ";
		_ballX = bluetooth->read(robotNumber);// cout << (int)_ballX << " ";
		_ballY = bluetooth->read(robotNumber);// cout << (int)_ballY << " ";
		friendPassState = bluetooth->read(robotNumber);
		//byte = bluetooth->read(robotNumber); cout << (int)byte << " ";


		if (friendX > 0 && friendY > 0){
			friendXY[0] = friendX; friendXY[1] = friendY;
			isFriendKnowed = true;
		}
		else{
			isFriendKnowed = false;
		}
		if (_ballX > 0 && _ballY > 0 && isBallKnowedFriend == true && myIsBallKnowed == false){
				ballX = _ballX; ballY = _ballY;
				isBallKnowed = true;
		}
	}
}
void  Robot::writeBluetooth(){
	//cout << "BLUETOOTHwrite: ";
	bluetooth->write(-31, robotNumber);// cout << -31<<" ";//pi
	bluetooth->write(-41, robotNumber);// cout << -41 << " ";//pi
	bluetooth->write(robotX, robotNumber);// cout << robotX << " ";
	bluetooth->write(robotY, robotNumber);// cout << robotY << " ";
	bluetooth->write(myIsBallKnowed, robotNumber);// cout << isBallKnowed << " ";
	bluetooth->write(ballX, robotNumber);// cout << ballX << " ";
	bluetooth->write(ballY, robotNumber);// cout << ballY << " ";
	bluetooth->write(myPassState, robotNumber);// cout << myPassState << " ";
	
}

char mergeBoolBluetooth(bool b1, bool b2, bool b3, bool b4, bool b5, bool b6, bool b7, bool b8){
	char _byte = 0;
	_byte = _byte | (b1 << 7);
	_byte = _byte | (b2 << 6);
	_byte = _byte | (b3 << 5);
	_byte = _byte | (b4 << 4);
	_byte = _byte | (b5 << 3);
	_byte = _byte | (b6 << 2);
	_byte = _byte | (b7 << 1);
	_byte = _byte | (b8 << 0);
	return _byte;
}

//---------- Solenoid ----------//
void  Robot::kick(){
#if !defined(ARDUINO)
	cout << "KICK!" << endl;
	simxSetIntegerSignal(clientIDSimulation, "active", 1, simx_opmode_oneshot);
	float ballPos[3] = { 0.120, -0.076, 0 };//SimulationSuctionPadHandle
	if (simxSetObjectParent(clientIDSimulation, SimulationSuctionPointHandle, -1, true, sim_handleflag_assembly) != simx_return_ok)
		cout << "Kick fail" << endl;
	simxSetObjectPosition(clientIDSimulation, SimulationBallHandle, SimulationRobotHandle, ballPos, simx_opmode_oneshot);
	solenoid->kick();
	kicked = false;
#endif
}

//---------- Servo ----------//
void Robot::updateServo(){

	int ret = 0;
	int message = -1;
	ret = simxGetInMessageInfo(clientIDSimulation, simx_headeroffset_server_state, &message);
	if (!((message & 2) == 2 || (message & 1) == 0))//if it has been paused
	{
		message = -1;
		ret = simxGetInMessageInfo(clientIDSimulation, simx_headeroffset_server_state, &message);
		

	//-----Update Camera Values-----//
	camera->updateCamera();
	if (robotWithBall == false){
		float ObjY = camera->getObjectY("ball");
		int FieldOfViewVertical = camera->getFieldOfViewV();
		int _servoAngle = servo->getAngle();

		if (GetIsBallKnowed())
		{
			float errorY = (0.5 - ObjY)*(-1);

			if (abs(errorY) > 0.05 && _servoAngle < 90)
			{
				_servoAngle += (FieldOfViewVertical / 2)*errorY;
			}
			if (_servoAngle < 50)_servoAngle = 50;

				servo->setAngle(_servoAngle);
				SetServoAngle(_servoAngle);
		}
		else
		{
			servo->setAngle(60);
			SetServoAngle(60);
		}
	}
	else{
		servo->setAngle(60);
		SetServoAngle(60);
	}
	}
}

//---------- Ultrasonic ----------//
Ultrasonic Robot::relativeSonar(int sonarNumber){

	int spinTimes = 0;//Number of sonar spins|EX: spin=2 -> Sonar 1 turns 3/Sonar 2 turns 4
	int spinTime_auxiliary = 1;
	int relativeNumberSonar;//The relative number after degree tratament

	if (compassValue <= 180)
	{
		while (compassValue - (15 + 30 * spinTimes) >= 0)
		{
			spinTimes++;
		}
	}
	else
	{
		while ((360 - compassValue) - (15 + 30 * spinTimes) <= 360)
		{
			spinTimes--;

		}
	}

	if (spinTimes < 0)
	{
		spinTimes = spinTimes*(-1);
		if (spinTimes == 13)
			spinTimes = 0;
		else
			spinTimes--;
	}
	spinTimesSonar = spinTimes;
	//cout << "(spin:" << spinTimes<<")";

	relativeNumberSonar = sonarNumber - spinTimes;
	while (relativeNumberSonar > 11)
		relativeNumberSonar -= 12;
	while (relativeNumberSonar < 0)
		relativeNumberSonar += 12;

	//return ultrasonics[relativeNumberSonar]->value();
	return *ultrasonics[relativeNumberSonar];
}
void Robot::updateSonar()
{
	updateSonarValuesAngles();
	updateSonarPointX();
	updateSonarPointY();
}
void Robot::updateSonarValuesAngles()
{
	for (int i = 0; i < 12; i++)
	{
		//-------------------------------------Sonar spin
		int spinTimes = 0;//Number of sonar spins|EX: spin=2 -> Sonar 1 turns 3/Sonar 2 turns 4
		int spinTime_auxiliary = 1;
		int relativeNumberSonar;//The relative number after degree tratament

		if (compassValue <= 180)
		{
			while (compassValue - (15 + 30 * spinTimes) >= 0)
			{
				spinTimes++;
			}
		}
		else
		{
			while ((360 - compassValue) - (15 + 30 * spinTimes) <= 360)
			{
				spinTimes--;

			}
		}

		if (spinTimes < 0)
		{
			spinTimes = spinTimes*(-1);
			if (spinTimes == 13)
				spinTimes = 0;
			else
				spinTimes--;
		}
		//-------------------------------------update sonar angle
		float degreeError = compassValue;
		float sonarAngleValue = 0;

		if (degreeError > 180)
		{
			degreeError = degreeError - 360;
		}

		/*if (degreeError <= 180)
			sonarAngleValue = degreeError - (360 - i * 30);
		else*/
		sonarAngleValue = (i * 30)+ degreeError -(spinTimes * 30);//Normal Compass angle + Degree error - SpinAjust

		while (sonarAngleValue < 0)//SonarAngleDistortion can be negative
		{
			sonarAngleValue += 360;
		}
		while (sonarAngleValue >= 360)
		{
			sonarAngleValue -= 360;
		}

		sonarAngles[i] = sonarAngleValue;
		//-------------------------------------update sonar value
		Ultrasonic sonar = relativeSonar(i);
		sonarValues[i] = sonar.value();
	}
}
void Robot::updateSonarPointX()
{
	//-----------------sonar 0 analysis (345ฐ <-0ฐ-> 15ฐ)
	if (sonarAngles[0] < 180)
		sonarPointX[0] = sonarValues[0] * sin(sonarAngles[0]*M_PI / 180);
	else
		sonarPointX[0] = (-1)* sonarValues[0] * sin((360 - sonarAngles[0]) *M_PI / 180);
	//-----------------sonar 1 analysis (15ฐ <-30ฐ-> 45ฐ)
	sonarPointX[1] = sonarValues[1] * sin(sonarAngles[1]*M_PI / 180);
	//-----------------sonar 2 analysis (45ฐ <-60ฐ-> 75ฐ)
	sonarPointX[2] = sonarValues[2] * sin(sonarAngles[2]*M_PI / 180);
	//-----------------sonar 3 analysis (75ฐ <-90ฐ-> 105ฐ)
	if (sonarAngles[3] <= 90)
		sonarPointX[3] = sonarValues[3] * cos((90 - sonarAngles[3])*M_PI / 180);
	else
		sonarPointX[3] = (sonarValues[3] * cos((sonarAngles[3] - 90)*M_PI / 180));
	//-----------------sonar 4 analysis (105ฐ <-120ฐ-> 135ฐ)
	sonarPointX[4] = (sonarValues[4] * cos((sonarAngles[4] - 90) *M_PI / 180));
	//-----------------sonar 5 analysis (135ฐ <-150ฐ-> 165ฐ)
	sonarPointX[5] = (sonarValues[5] * cos((sonarAngles[5] - 90) *M_PI / 180));
	//-----------------sonar 6 analysis(165ฐ <-180ฐ-> 195ฐ)
	if (sonarAngles[6] < 180)
		sonarPointX[6] = sonarValues[6] * sin((180 - sonarAngles[6]) *M_PI / 180);
	else
		sonarPointX[6] = (-1)* sonarValues[6] * sin((sonarAngles[6] - 180) *M_PI / 180);
	//-----------------sonar 7 analysis(195ฐ <-210ฐ-> 225ฐ)
	sonarPointX[7] = (-1)*(sonarValues[7]) * cos(((360 - sonarAngles[7]) - 90)*M_PI / 180);
	//-----------------sonar 8 analysis(225ฐ <-240ฐ-> 255ฐ)
	sonarPointX[8] = (-1)*(sonarValues[8]) * cos(((360 - sonarAngles[8]) - 90)*M_PI / 180);
	//-----------------sonar 9 analysis(255ฐ <-270ฐ-> 285ฐ)
	if (sonarAngles[9] <= 270)
		sonarPointX[9] = (-1)* (sonarValues[9]) * cos(((360 - sonarAngles[9]) - 90)*M_PI / 180);
	else
		sonarPointX[9] = (-1)* (sonarValues[9]) * cos((sonarAngles[9] - 270)*M_PI / 180);
	//-----------------sonar 10 analysis(285ฐ <-300ฐ-> 315ฐ)
	sonarPointX[10] = (-1)* (sonarValues[10]) * cos((sonarAngles[10] - 270)*M_PI / 180);
	//-----------------sonar 11 analysis(315ฐ <-330ฐ-> 345ฐ)
	sonarPointX[11] = (-1)* (sonarValues[11]) * cos((sonarAngles[11] - 270)*M_PI / 180);
}
void Robot::updateSonarPointY()
{
	//-----------------sonar 0 analysis (345ฐ <-0ฐ-> 15ฐ)
	if (sonarAngles[0] < 180)
		sonarPointY[0] = sonarValues[0] * cos(sonarAngles[0] * M_PI / 180);
	else
		sonarPointY[0] = sonarValues[0] * cos((360 - sonarAngles[0]) *M_PI / 180);
	//-----------------sonar 1 analysis (15ฐ <-30ฐ-> 45ฐ)
	sonarPointY[1] = sonarValues[1] * cos(sonarAngles[1] * M_PI / 180);
	//-----------------sonar 2 analysis (45ฐ <-60ฐ-> 75ฐ)
	sonarPointY[2] = sonarValues[2] * cos(sonarAngles[2] * M_PI / 180);
	//-----------------sonar 3 analysis (75ฐ <-90ฐ-> 105ฐ)
	if (sonarAngles[3] <= 90)
		sonarPointY[3] = sonarValues[3] * sin((90 - sonarAngles[3])*M_PI / 180);
	else
		sonarPointY[3] = (-1)*(sonarValues[3] * sin((sonarAngles[3] - 90)*M_PI / 180));
	//-----------------sonar 4 analysis (105ฐ <-120ฐ-> 135ฐ)
	sonarPointY[4] = (-1)*(sonarValues[4] * sin((sonarAngles[4] - 90) *M_PI / 180));
	//-----------------sonar 5 analysis (135ฐ <-150ฐ-> 165ฐ)
	sonarPointY[5] = (-1)*(sonarValues[5] * sin((sonarAngles[5] - 90) *M_PI / 180));
	//-----------------sonar 6 analysis(165ฐ <-180ฐ-> 195ฐ)
	if (sonarAngles[6] < 180)
		sonarPointY[6] = (-1)*sonarValues[6] * cos((180 - sonarAngles[6]) *M_PI / 180);
	else
		sonarPointY[6] = (-1)* sonarValues[6] * cos((sonarAngles[6] - 180) *M_PI / 180);
	//-----------------sonar 7 analysis(195ฐ <-210ฐ-> 225ฐ)
	sonarPointY[7] = (-1)*(sonarValues[7]) * sin(((360 - sonarAngles[7]) - 90)*M_PI / 180);
	//-----------------sonar 8 analysis(225ฐ <-240ฐ-> 255ฐ)
	sonarPointY[8] = (-1)*(sonarValues[8]) * sin(((360 - sonarAngles[8]) - 90)*M_PI / 180);
	//-----------------sonar 9 analysis(255ฐ <-270ฐ-> 285ฐ)
	if (sonarAngles[9] <= 270)
		sonarPointY[9] = (-1)* (sonarValues[9]) * sin(((360 - sonarAngles[9]) - 90)*M_PI / 180);
	else
		sonarPointY[9] = (sonarValues[9]) * sin((sonarAngles[9] - 270)*M_PI / 180);
	//-----------------sonar 10 analysis(285ฐ <-300ฐ-> 315ฐ)
	sonarPointY[10] = (sonarValues[10]) * sin((sonarAngles[10] - 270)*M_PI / 180);
	//-----------------sonar 11 analysis(315ฐ <-330ฐ-> 345ฐ)
	sonarPointY[11] = (sonarValues[11]) * sin((sonarAngles[11] - 270)*M_PI / 180);
}

//---------- Compass ----------//
void Robot::updateCompass()
{
	//----- Receive sensor values -----//
	compassValue = compass->value();

	//----- Calculate XY(gyro/accelerometer) -----//
#if !defined(ARDUINO)
	float posSimulation[3] = { 0 };

	simxGetObjectPosition(clientIDSimulation, SimulationRobotHandle, -1, posSimulation, simx_opmode_streaming);

	if (robotNumber % 2 == 0){
		SetRobotX(91 - posSimulation[0] * 100);
		SetRobotY(121.5 - posSimulation[1] * 100);
	}
	else{
		SetRobotX(91 + posSimulation[0] * 100);
		SetRobotY(121.5 + posSimulation[1] * 100);
	}


	SetIsRobotXKnowed(true);
	SetIsRobotYKnowed(true);
#endif
}

//---------- Robot ----------//
void Robot::updateRobotX(){
	bool localDebug = false;

	if (localDebug)
		printf("DEBUG robot X:\n");

	int widthField=fieldMatrix->getWidthField();
	int lengthField = fieldMatrix->getLengthField();

	int possibleValues[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//Receives sonar 1,2,3,4,5|7,8,9,10,11 (start in 0) - front sonar and back sonar are ignored
	int possibleX_Auxiliary[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //each position one increases by 1 each time it is close to another value
	int possibleX_AuxiliaryGrowing[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	int bestValuesSum = 0;//sum of best values to calculate X
	int numberOfBest = 0;//amount of best values

	int possibleX = 0;//Return value

	//-----------------sonar 1 analysis
	possibleValues[0] = widthField - sonarPointX[1];
	//-----------------sonar 2 analysis
	possibleValues[1] = widthField - sonarPointX[2];
	//-----------------sonar 3 analysis
	possibleValues[2] = widthField - sonarPointX[3];
	//-----------------sonar 4 analysis
	possibleValues[3] = widthField - sonarPointX[4];
	//-----------------sonar 5 analysis
	possibleValues[4] = widthField - sonarPointX[5];
	//-----------------sonar 7 analysis
	possibleValues[5] = (-1)*sonarPointX[7];//sonarPointX is negative
	//-----------------sonar 8 analysis
	possibleValues[6] = (-1)*sonarPointX[8];//sonarPointX is negative
	//-----------------sonar 9 analysis
	possibleValues[7] = (-1)*sonarPointX[9];//sonarPointX is negative
	//-----------------sonar 10 analysis
	possibleValues[8] = (-1)*sonarPointX[10];//sonarPointX is negative
	//-----------------sonar 11 analysis
	possibleValues[9] = (-1)*sonarPointX[11];//sonarPointX is negative

	
	if (localDebug)
	{
		printf("\tpossible X: ");
		for (int i = 0; i < 10; i++)
		{
			printf("[%d]", possibleValues[i]);
		}
		printf("\n");

	}

	if (isFirstRobotX == false)
	{
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				if (isNear(possibleValues[i], possibleValues[j], 7) && isNear(possibleValues[i], GetRobotX(), GetDiscrepancyX()))
				{
					possibleX_Auxiliary[i]++;
				}
			}
		}
	}
	else
	{
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				if (isNear(possibleValues[i], possibleValues[j], 4))
				{
					possibleX_Auxiliary[i]++;
				}
			}
		}
		isFirstRobotX = false;
	}
	for (int i = 0; i < 10; i++)
	{
		possibleX_AuxiliaryGrowing[i] = possibleX_Auxiliary[i];
	}
	risingVector(possibleX_AuxiliaryGrowing, 10);


	if (localDebug)
	{
		printf("\tpossibleX_Auxiliary: ");
		for (int i = 0; i < 10; i++)
		{
			printf("[%d]", possibleX_Auxiliary[i]);
		}
		printf("\n");
	}
	///------------------------------------see values
	/*//DEBUG 1000ms
	debug1.confirmTimeMilliseconds(1000);//was bugging
	debug1.update();
	if (debug1.executeThread() == true)
	{
	cout << "possibleValuesX= " << "[";
	for (int i = 0; i < 10; i++)
	{
	cout << possibleValues[i];
	if (i < 10 - 1)
	{
	cout << ",";
	}
	}
	cout << "]" << endl;

	cout << "possibleX_Auxiliary= " << "[";
	for (int i = 0; i < 10; i++)
	{
	cout << possibleX_Auxiliary[i];
	if (i < 10 - 1)
	{
	cout << ",";
	}
	}
	cout << "]" << endl;
	}//*/
	//----------------------------------------------positionX tratament
	if (possibleX_AuxiliaryGrowing[9] > 2)
	{
		SetIsRobotXKnowed(true);//Know X

		for (int i = 0; i < 10; i++)
		{
			if (possibleX_Auxiliary[i] == possibleX_AuxiliaryGrowing[9])
			{
				bestValuesSum += possibleValues[i];
				numberOfBest++;
			}
		}

		//cout << "bestValuesSum= " << bestValuesSum << endl;
		//cout << "numberOfBest= " << numberOfBest << endl;

		possibleX = int(bestValuesSum / numberOfBest);
		//cout << "I think is " << possibleX << endl;
		if (isFirstRobotX == true)
		{
			SetRobotX(possibleX);
			isFirstRobotX = false;
		}


		if (!isNear(possibleX, GetRobotX(), discrepancyX))
		{
			//cout << possibleX << " is more than " << discrepancyTime << " distant of " << lastRobotX << "	:(" << endl;
			SetIsRobotXKnowed(false);//dont know X
		}
		else
		{
			discrepancyX = 10;
			SetIsRobotXKnowed(true);
		}
		if (GetIsRobotXKnowed() == true)
		{
			SetRobotX(possibleX);
		}
	}
	else
		SetIsRobotXKnowed(false);//dont know X

	if (GetIsRobotXKnowed() == false)
	{
		//cout << "I don't know the position X value" << endl;
		increaseDiscrepancyX->setTimeMilliseconds(100);
		if (increaseDiscrepancyX->executeThread() == true)
		{
			int lostCycles = (increaseDiscrepancyX->lastDiscrepancy) / 100;
			discrepancyX += 5 * lostCycles;
		}
	}
}
void Robot::updateRobotY(){
	bool localDebug = false;

	if (localDebug)
		printf("Debug robot Y:\n");

	updateSonarPointY();
	updateSonarPointX();

	int widthField = fieldMatrix->getWidthField();
	int lengthField = fieldMatrix->getLengthField();

	int possibleValues[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float ValuesProbability[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//probability 0 to 1

	float  ProbabilityGrowing[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//probability growing
	int SonarIndexProbabilityGrowing[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//sonar index of probability growing

	//COMEวAR A ANALISAR DO ฺLTIMO, SE ANALISAR COM O ULTIMO VALOR E FOR MUITO DISCREPANTE AI PASSA PARA O PRำXIMO
	//SE ENCONTRAR UM QUE SATISFAZ A CONDIวรO Jม ษ O robotY
	//SE NรO ENCONTRAR NENHUM ENTรO DECRETA isYKnowed=false

	int possibleY = 0;//More probable
	for (int i = 0; i < 12; i++)
	{
		int j = i;
		if (i > 3)//jump the three (sonar[2] is sonar two AND sonar[3] is sonar four) (just for possibleValues)
			j--;
		if (i > 9)//jump the nine (sonar[7] is sonar eight AND sonar[8] is sonar ten) (just for possibleValues)
			j--;
		if ((robotX + sonarPointX[i] > widthField - 5) || (robotX + sonarPointX[i] <5))
			possibleValues[j] = 0;//ignore this value (probabily is a lateral wall)
		else if (((robotX + sonarPointX[i] <=(widthField / 2) + 30) && (robotX + sonarPointX[i] >=(widthField / 2) - 30)) && 
			((robotY + sonarPointY[i] < 239) && (robotY + sonarPointY[i] > 5)))//adversary goal
		{
			if ((robotX + sonarPointX[i] <=(widthField / 2) + 30) && (robotX + sonarPointX[i] >=(widthField / 2) + 29))
				possibleValues[j] = 0;//ignore this value (probabily is a lateral wall)
			else if ((robotX + sonarPointX[i] >=(widthField / 2) - 30) && (robotX + sonarPointX[i] <=(widthField / 2) - 29))
				possibleValues[j] = 0;//ignore this value (probabily is a lateral wall)
			else
			{
				//Sonar seeing the goal
				//Calculate sonar number (high sonars see the beam)
				int sonarNumber = i - spinTimesSonar;
				sonarNumber < 0 ? sonarNumber += 12 :
					(sonarNumber>11 ? sonarNumber -= 12 : sonarNumber = sonarNumber);

				if ((sonarNumber == 0) || (sonarNumber == 1) || (sonarNumber == 5) || (sonarNumber == 7) || (sonarNumber == 11))
				{
					if (i<3 || i>9)
						possibleValues[j] = lengthField - (30 + sonarPointY[i]);
					else
						possibleValues[j] = 30 + (-1)*sonarPointY[i];

				}
				else
				{
					if (i < 3 || i>9)
						possibleValues[j] = lengthField - ((30 - 7) + sonarPointY[i]);
					else
					{
						possibleValues[j] = ((30 - 7) + (-1)*sonarPointY[i]);
					}
				}
			}
		}
		else
		{
			if (i<4 || i>8)
				possibleValues[j] = lengthField - sonarPointY[i];
			else
				possibleValues[j] = (-1)*sonarPointY[i];
		}
		/*else if ((robotX + sonarPointX[i] <(widthField / 2) + 33) && (robotX + sonarPointX[i] >(widthField / 2) + 27))
			possibleValues[j] = 0;//ignore this value (probabily is a lateral wall)
		else if ((robotX + sonarPointX[i] <(widthField / 2) - 27) && (robotX + sonarPointX[i] >(widthField / 2) - 33))
			possibleValues[j] = 0;//ignore this value (probabily is a lateral wall)
		else if ((robotX + sonarPointX[i] > (widthField / 2) + 30) || (robotX + sonarPointX[i] < (widthField / 2) - 30))
		{
			if (i<4 || i>8)
				possibleValues[j] = lengthField - sonarPointY[i];
			else
				possibleValues[j] = (-1)*sonarPointY[i];
		}
		else
		{
			//Sonar seeing the goal
			//Calculate sonar number (high sonars see the beam)
			int sonarNumber = i - spinTimesSonar;
			sonarNumber < 0 ? sonarNumber += 12 :
				(sonarNumber>11 ? sonarNumber -= 12 : sonarNumber = sonarNumber);

			if ((sonarNumber == 0)||(sonarNumber == 1) || (sonarNumber == 5) || (sonarNumber == 7) || (sonarNumber == 11))
			{
				if (i<3 || i>9)
					possibleValues[j] = lengthField - (30 + sonarPointY[i]);
				else
					possibleValues[j] = 30 + (-1)*sonarPointY[i];
				
			}
			else
			{
				if (i < 3 || i>9)
					possibleValues[j] = lengthField - ((30 - 7) + sonarPointY[i]);
				else
				{
					possibleValues[j] = ((30 - 7) + (-1)*sonarPointY[i]);
				}
			}
		}*/
		if (i == 2 || i == 8)
			i++;
	}

	int bestFrontSonar = 0; //best front ultrassonic index
	int bestBackSonar = 5;	 //best back ultrassonic index

	int numbersFrontReading = 0;//numbers of front sonar with possible reading!
	int numbersBackReading = 0;//numbers of back sonar with possible reading!

	int bestSonarsSum = 0;//sum of the ultrassonics values
	int bestSonarsNumber = 0;//number of ultrassonics values in the sum
	//-----choose best front (8-9-0-1-2)-----//
	if (localDebug)
		printf("\tReadings front: ");

	int bestFrontSonarVal = 243;//used because sonar 0 can return 0 -> bug

	for (int i=0; i < 10; i++)//best front sonar
	{
		if (localDebug)
		{
			if (possibleValues[i] <= bestFrontSonarVal && possibleValues[i]>0)
				printf("[%d*]", possibleValues[i]);
			else
				printf("[%d]", possibleValues[i]);
		}

		if (possibleValues[i] <= bestFrontSonarVal && possibleValues[i]>0)
		{
			bestFrontSonar = i;
			bestFrontSonarVal = possibleValues[i];
		}
		if (i == 2)
			i = 7;
	}

	if (localDebug)
		printf("\n");

	for (int i = 0; i < 10; i++)//number of sensors that agree with the best sonar
	{
		if (isNear(possibleValues[i],possibleValues[bestFrontSonar],5))
		{
			numbersFrontReading++;
		}
		if (i == 2)
			i = 7;
	}
	//-----choose best back (7-6-5-4-3)-----//
	if (localDebug)
		printf("\tReadings back: ");

	int bestBackSonarVal = 0;//used because this is used in front sonars

	for (int i = 3; i < 8; i++)//best back sonar
	{
		if (localDebug)
		{
			if (possibleValues[i] >= bestBackSonarVal && possibleValues[i]>0)
				printf("[%d*]", possibleValues[i]);
			else
				printf("[%d]", possibleValues[i]);
		}

		if (possibleValues[i] >= bestBackSonarVal && possibleValues[i]>0)
		{
			bestBackSonar = i;
			bestBackSonarVal = possibleValues[i];
		}
	}

	if (localDebug)
		printf("\n");

	for (int i = 3; i < 8; i++)//number of sensors that agree with the best sonar
	{
		if (isNear(possibleValues[i], possibleValues[bestBackSonar], 5))
		{
			numbersBackReading++;
		}
	}
	//printf("\tBF:%d(%d) BB:%d(%d)\n", possibleValues[bestFrontSonar], bestFrontSonar, possibleValues[bestBackSonar], bestBackSonar);
	//-----sum and mean of best values-----//
	SetIsRobotYKnowed(false);
	if ((isNear(possibleValues[bestFrontSonar], GetRobotY(), GetDiscrepancyY())) && (possibleValues[bestFrontSonar]>0))
	{
		if (numbersFrontReading - numbersBackReading >= 0)
		{
			bestSonarsSum += possibleValues[bestFrontSonar];
			bestSonarsNumber++;
			discrepancyY = 10;
			SetIsRobotYKnowed(true);
		}
	}

	if ((isNear(possibleValues[bestBackSonar], GetRobotY(), GetDiscrepancyY())) && (possibleValues[bestBackSonar]>0))
	{
		if (numbersBackReading - numbersFrontReading >= 0)
		{
			bestSonarsSum += possibleValues[bestBackSonar];
			bestSonarsNumber++;
			discrepancyY = 10;
			SetIsRobotYKnowed(true);
		}
	}
	//-----return mean-----//
	if (GetIsRobotYKnowed() == true)
	{
		int lastY = GetRobotY();
		int Yprobably = (bestSonarsSum / bestSonarsNumber);

		SetRobotY(Yprobably);

		updateSonarPointX();
		updateSonarPointY();

		for (int i = 0; i < 12; i++)//check if there is a sonar with negative Ypoint
		{
			if (sonarPointY[i]+GetRobotY() < 0)
			{
				while (sonarPointY[i] + Yprobably < 0)
					Yprobably++;
				SetRobotY(Yprobably);//this value isn't possible, return to the last
				//SetIsRobotYKnowed(false);
			}
			if (sonarPointY[i] + GetRobotY() > lengthField)
			{
				while (sonarPointY[i] + Yprobably > lengthField)
					Yprobably--;
				SetRobotY(Yprobably);//this value isn't possible, return to the last
				//SetIsRobotYKnowed(false);
			}
		}
	}
	if (GetIsRobotYKnowed() == false)
	{
		//cout <<"I don't know the position Y value" << endl;
		increaseDiscrepancyY->setTimeMilliseconds(100);
		if (increaseDiscrepancyY->executeThread() == true)
		{
			int lostCycles = (increaseDiscrepancyY->lastDiscrepancy) / 100;
			discrepancyY += 5 * lostCycles;
		}
	}

}

void Robot::updateRobotAngle(){
	float angleCompass = -compassValue;

	if (isBallKnowed){
		frontAngle = angleCompass + (camera->getFieldOfViewH() / 2)*(0.5 - ballXcam);
		frontAngle < 0 ? frontAngle += 360 : frontAngle;

		if (robotWithBall && isGoalKnowed){
			float minorAngle = angleCompass + (float(camera->getFieldOfViewH()) / 2)*(0.5 - goalXCam /*-(goalLengthCam / 2) + (8 / ratioValues)*/);
			frontAngle = minorAngle;
		}
		else if (robotWithBall){
			float angleToCenterGoal = angleTwoPoints(robotX, robotY, 91, 243 - 30);
			frontAngle = angleToCenterGoal;
		}
	}
	else{
		frontAngle = 0;
		/*firstUnknowBall = false;
		if (firstUnknowBall){//start rotating for the side that the ball left
			isBallRight == true ? clockWise = true : clockWise = false;
			firstUnknowBall = false;
			cout << "First Unkow" << endl;
		}
		else{
			frontAngle < 0 ? frontAngle += 360 : frontAngle;
			frontAngle >= 360 ? frontAngle -= 360 : frontAngle;//angle [0,360[
			//frontAngle > 180 ? frontAngle - 360 : frontAngle;//angle ]-180,180]


			if (robotY > 100)
			{
				cout << "Searching 100degree" << endl;
				if (clockWise == true){
					frontAngle = 40;
					if (isNearAngle(-compassValue, frontAngle, 3))
						clockWise = false;
				}
				else if (clockWise == false){
					frontAngle = -40;
					if (isNearAngle(-compassValue, frontAngle, 3))
						clockWise = true;
				}
			}
			else{
				cout << "Searching 90degree";
				if (clockWise == true){
					frontAngle = 30;
					cout << "(30)" << -compassValue << " " << frontAngle << endl;
					if (isNearAngle(-compassValue, frontAngle, 3))
						clockWise = false;
				}
				else if (clockWise == false){
					frontAngle = -30;
					cout << "(-30)" << -compassValue << " " << frontAngle << endl;
					if (isNearAngle(-compassValue, frontAngle, 3))
						clockWise = true;
				}
				//}
			}
		}*/
	}
}
//---------- Adversary ----------//
void Robot::updateAdversary(){
	for (int i = 0; i < 25; i++){ adversaryXY[i] = 0; }
	numAdversaries = 0;
	//int obstableY[12];		for (int i = 0; i < 12; i++){ obstableY[i] = 0; }
	//int numberObstacles = 0;
	//cout << "Obstacles:";
	for (int i = 0; i < 12; i++)
	{
		int readX = GetRobotX() + sonarPointX[i];
		int readY = GetRobotY() + sonarPointY[i];
		if (readX>30 && readX < 152)//if is reading the wall isn't obstacle		(40cm border)
		{
			if (readY>40 && readY < 193)//if is reading the wall isn't obstacle	(40cm border)
			{
				if (readX>51 && readX < 131) //if the obstacle can be the goal	(3cm border)
				{
					if ((readY>19 && readY < 37) || (readY>203 && readY < 224))// is the goal!! (3cm border size)(7cm border front)
					{
						goto isntObstacle;
					}
				}
				numAdversaries++;
				adversaryXY[(numAdversaries - 1) * 2 + 1] = readX;//1,3,5,7,9
				adversaryXY[(numAdversaries - 1) * 2 + 2] = readY;//2,4,6,8,10
				//cout << "(" << i << ")";
			}
		}
	isntObstacle:;
	}
	/*cout << endl;
	for (int i = 0; i < 12; i++)
	{
	cout << "[" << adversaryXY[(i - 1) * 2 + 1] << "," << adversaryXY[(i - 1) * 2 + 2] << "]";
	}*/
}

//---------- Camera ----------//
void Robot::updateCamera(){
	SetIsBallKnowed(true);
	myIsBallKnowed = true;

	camera->updateCamera();
	float ballNumber = camera->getNumberObjects("ball");
	float ballX = camera->getObjectX("ball"); ballXcam = ballX;

	float ballY = camera->getObjectY("ball"); ballYcam = ballY;
	double ballSize = camera->getObjectSize("ball");

	float goalBNumber = camera->getNumberObjects("goalb");
	float goalYNumber = camera->getNumberObjects("goaly");

	float goalX = camera->getObjectX("goal");
	float goalY = camera->getObjectY("goal");
	float goalLength = camera->getObjectLength("goal");
	float goalHeight = camera->getObjectHeight("goal");

	if ((goalBNumber + goalYNumber) != 0)
		isGoalKnowed = true;
	else
		isGoalKnowed = false;



	int FieldOfViewHorizontal = camera->getFieldOfViewH();
	int FieldOfViewVertical = camera->getFieldOfViewV();
	//-------------------- BALL --------------------//
	//----- Ball exception-----//
	//don't calculate ball value if is too close to the border view (usually inaccurate)
	if ((ballSize / 2) > ballX || (ballSize / 2) > 1 - ballX){
		SetIsBallKnowed(false);
		myIsBallKnowed = false;
	}
		
	//-----Variables-----//
	int ballDistance = 0;
	float Kd = 0.207 * 20 / GetBallDiameter();
	int robotCamHeight = 18;//centimeters above the ground
	int currentServoAngle = GetServoAngle();
	//-----Calculate compensation Size-----//
	//to calculate this use the error proportion when the ball is 30cm far the robot
	double SizeCenter = 0.148762894847;//size of the ball when 30cm far the robot
	double objXsquare = ballX*ballX;
	double SizeComp = 0.178270547464*objXsquare - 0.190693118212*ballX + 0.199541817087;//possible size ball with actual X when 30cm far the robot
	double SizeAdjust = SizeCenter / SizeComp;//calculate the adjust value to size be constant with Xobj change
	ballSize *= SizeAdjust;
	//-----Calculation-----//
	if ((ballNumber == 0) || (GetIsRobotXKnowed() == false))
	{
		SetIsBallKnowed(false);
		myIsBallKnowed = false;
		SetBallX(0);
		SetBallY(0);
	}
	else if (GetIsBallKnowed() == true)
	{
		myIsBallKnowed = true;
		SetIsBallKnowed(true);
		//-----Calculation Ball distance-----//
		ballDistance = (ballDiameter*Kd / ballSize);

		ballX >= 0.5 ? isBallRight = true : isBallRight = false;

		double errorX = abs(0.5 - ballX);
		double errorY = abs(0.5 - ballY);

		if ((errorX < 0.15 && errorY < 0.15) && ballDistance < 85)//if the ball is in the center can use this method (more accurate)
		{
			float totalCamHeight = robotCamHeight;//change this in the future
			float XTotalDistance = tan(currentServoAngle*M_PI / 180)*totalCamHeight;
			float XExceededDistance = tan(currentServoAngle*M_PI / 180)*GetBallDiameter() / 2;

			//ballDistance = XTotalDistance - XExceededDistance;
		}
		//-----Calculation Ball XY-----//

		float ballAngle = -compassValue + FieldOfViewHorizontal*(0.5 - ballX);
		int relativeBallX = ballDistance*sin(ballAngle*M_PI / 180);
		int relativeBallY = ballDistance*cos(ballAngle*M_PI / 180);
		SetBallX(GetRobotX() - relativeBallX);
		SetBallY(GetRobotY() + relativeBallY + 6);//the camera is 6cm from the center of the robot
	}
	//-------------------- GOAL --------------------//
	if (isGoalKnowed){
		realGoalLength = realGoalHeight / goalHeight*goalLength;


		compassValue >= 360 ? compassValue -= 360 : compassValue;
		compassValue < 0 ? compassValue += 360 : compassValue;

		if (compassValue > 240 || compassValue < 120){
			goalLengthCam = goalLength;
			goalHeightCam = goalHeight;
			goalXCam = camera->getObjectX("goal");
			goalYCam = camera->getObjectY("goal");
		}
		else{
			goalLengthCam = 0;
			goalXCam = 0;
			goalYCam = 0;
			realGoalLength = 0;
			isGoalKnowed = false;

		}
		//cout << "GOAL x:" << goalXCam << " y:" << goalYCam << " l:" << goalLength << " h:" << goalHeight << " Rl:" << realGoalLength;
	}
}

void Robot::updateLightSensor()
{
	static bool FirstBall = true;
	//float ballValue = lightBallPos->value();
	int value[1];
	simxGetObjectChild(clientIDSimulation, SimulationBallHandle, 0, value, simx_opmode_oneshot);
	if (value[0] == SimulationSuctionPointHandle){//simulation suction with the ball
		if (FirstBall == true){
			FirstBall = false;
			float ballPos[3] = { 0.120, -0.086, 0 };//SimulationSuctionPadHandle{ 0.120, -0.086, 0 }
			//simxSetObjectPosition(clientIDSimulation, SimulationBallHandle, SimulationRobotHandle, ballPos, simx_opmode_oneshot);
		}
		SetRobotWithBall(true);
		teamWithBall = true;
	}
	else{
		FirstBall = true;
		simxSetObjectParent(clientIDSimulation, SimulationSuctionPointHandle, SimulationSuctionPadHandle, true, sim_handleflag_assembly);
		SetRobotWithBall(false);
		teamWithBall = false;
	}
}
//---------- Simulation ----------//
void Robot::initializeSimulation(int ID){
	clientIDSimulation = ID;
	cout << "---------- Connecting Robot " << robotNumber << " ----------" << endl;

	string robotName = "Soccer_Robot#" + std::to_string(robotNumber);

	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)robotName.c_str(), (simxInt *)&SimulationRobotHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << robotName << "- robot" << robotNumber << " not found!" << std::endl;
	else
		cout << "Connected to the " << robotName << "- robot" << robotNumber << std::endl;

	if (simxGetObjectHandle(clientIDSimulation, "Ball", &SimulationBallHandle, simx_opmode_blocking) != simx_return_ok)
		cout << "Ball not found!" << endl;
	else
		cout << "Connected to the Ball" << endl;

	string suctionPointName = "suctionPadLoopClosureDummy1#" + std::to_string(robotNumber);

	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)suctionPointName.c_str(), (simxInt *)&SimulationSuctionPointHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << suctionPointName << "- robot" << robotNumber << " not found!" << std::endl;
	else
		cout << "Connected to the " << suctionPointName << "- robot" << suctionPointName << std::endl;

	string suctionPadName = "suctionPad#" + std::to_string(robotNumber);

	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)suctionPadName.c_str(), &SimulationSuctionPadHandle, simx_opmode_blocking) != simx_return_ok)
		cout << "SuctionPad not found!" << endl;
	else
		cout << "Connected to the SuctionPad" << endl;
	
	//motors
	for (int i = 0; i < 4; i++)
	{
		motors[i]->initializeSimulation(ID);
		motors[i]->power(0);
	}
	motorXicoR->initializeSimulation(ID);
	motorXicoR->power(0);
	//Ultrasonics
	for (int i = 0; i < 12; i++)
	{
		ultrasonics[i]->initializeSimulation(ID);
	}
	//Compass
	compass->initializeSimulation(ID);
	//LightSensor
	lightBallPos->initializeSimulation(ID);
	//Camera
	camera->initializeSimulation(ID);
	//Servo
	servo->initializeSimulation(ID);
	servo->setAngle(60);
	//Solenoid
	solenoid->initializeSimulation(ID);
}

//---------- Field monitoring ----------//
void Robot::drawSonarPoints()
{
	int robotX = GetRobotX();
	int robotY = GetRobotY();
	int widthField = fieldMatrix->getWidthField();
	int lengthField = fieldMatrix->getLengthField();

	sonarCleaning();

	int XPoint;
	int YPoint;

	if (GetIsRobotXKnowed() == true && GetIsRobotYKnowed() == true)
	{
		for (int i = 0; i < 12; i++)
		{
			XPoint = robotX + sonarPointX[i];
			YPoint = robotY + sonarPointY[i];
			if (XPoint < 0)
				XPoint = 0;
			if (XPoint >= widthField)
				XPoint = widthField - 1;
			if (YPoint < 0)
				YPoint = 0;
			if (YPoint >= lengthField)
				YPoint = lengthField - 1;

			fieldMatrix->pointWrite(XPoint, YPoint, true);
		}
	}
}
void Robot::delAlonePoints()
{

}
void Robot::sonarCleaning()
{
	int eraseCentimeters = 10;//erase Xcm | EX: (robot)----------------XXX (Obstacle) 

	int robotX = GetRobotX();
	int robotY = GetRobotY();
	for (int i = 0; i < 12; i++)
	{
		int sonarPX = robotX + sonarPointX[i];
		int sonarPY = robotY + sonarPointY[i];

		int lineX = robotX - sonarPX;//X=Xend-Xstart
		int lineY = robotY - sonarPY;//Y=Yend-Ystart
		float lineHip = sqrt(pow(lineX, 2) + pow(lineY, 2));

		float newLineHip = lineHip - eraseCentimeters;
		if (newLineHip > 0)//if newLineHip is negative, it doesn't make sense to erase
		{
			float relationHip = newLineHip / lineHip;
			int newlineX = robotX + sonarPointX[i] * relationHip;//new line end point
			int newlineY = robotY + sonarPointY[i] * relationHip;//new line end point

			fieldMatrix->bigLineWrite(robotX, robotY, newlineX, newlineY, 3, false);
		}
	}
}

//---------- Auxiliary functions ----------//
bool Robot::isNear(float firstNumber, float secondNumber, float discrepancy)//return false if discrepancy > distance of the numbers
{
	bool isNear = true;

	if (firstNumber - secondNumber > discrepancy)
		isNear = false;
	else if (secondNumber - firstNumber > discrepancy)
		isNear = false;

	return isNear;
}
bool Robot::isNearAngle(float firstNumber, float secondNumber, float discrepancy)//return false if discrepancy > distance of the numbers
{
	return (distanceTwoAngles(firstNumber,secondNumber) <= discrepancy);
}
float Robot::distanceTwoAngles(float firstNumber, float secondNumber)//return false if discrepancy > distance of the numbers
{

	float firstNumber180;//change the number (0 to 360) to ((-180) to (+180))
	float secondNumber180;
	float distanceNumbers;

	while (firstNumber >= 360)firstNumber -= 360;
	while (firstNumber < 0)firstNumber += 360;
	while (secondNumber >= 360)secondNumber -= 360;
	while (secondNumber < 0)secondNumber += 360;

	float dist10 = 0;
	float dist20 = 0;
	if (secondNumber >= firstNumber){
		dist10 = firstNumber;
		dist20 = 360 - secondNumber;
	}
	else{//(firstNumber > secondNumber)
		dist10 = secondNumber;
		dist20 = 360 - firstNumber;
	}
	distanceNumbers = dist10 + dist20;
	if (distanceNumbers > 180)
		distanceNumbers = 360 - distanceNumbers;
	return distanceNumbers;
}
bool Robot::isAngleTimeClock(float firstNumber, float secondNumber){
	int firstNumber180 = firstNumber - 180;//angle opposite 360

	while (firstNumber >= 360)firstNumber -= 360;
	while (firstNumber < 0)firstNumber += 360;
	while (firstNumber180 >= 360)firstNumber180 -= 360;
	while (firstNumber180 < 0)firstNumber180 += 360;
	while (secondNumber >= 360)secondNumber -= 360;
	while (secondNumber < 0)secondNumber += 360;

	if (firstNumber <= 180){
		if (secondNumber<firstNumber || secondNumber>firstNumber180)
			return true;
		else
			return false;
	}
	else if (firstNumber > 180){
		if (secondNumber<firstNumber && secondNumber>firstNumber180)
			return true;
		else
			return false;
	}
}
float Robot::angleTwoPoints(int x1, int y1, int x2, int y2)
{
	int angleResult=0;

	float triangleAngle = atan2((y2 - y1), (x2 - x1)) * 180 / M_PI;

	angleResult = triangleAngle - 90;
	
	if (angleResult < 0)
		angleResult += 360;
	if (angleResult > 360)
		angleResult -= 360;

	if (angleResult > 180)
		angleResult = (180-(angleResult - 180))*(-1); // ajust to return an value better to understand
												// 0 -> 90 -> 180 -> (-90) -> 0
	return angleResult;
}
float Robot::distanceTwoPoints(int x1, int y1, int x2, int y2)
{
	int powDeltaX = pow(x2 - x1, 2);
	int powDeltaY = pow(y2 - y1, 2);
	return sqrt(powDeltaX + powDeltaY);
}
void Robot::risingVector(int vector[], int length)
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

//---------- Train Robot ----------//
#if !defined(ARDUINO)
bool reachThePosition = true;
double xGoal = 0;
double yGoal = 0;
bool Robot::randomMoviment(vector<float> &realXY){
	float realRobotX = realXY[0] * 243;
	float realRobotY = realXY[1] * 243;
	if (reachThePosition == true){
		xGoal = (rand() % 163 + 10);//10cm to 172
		yGoal = (rand() % 164 + 40);//40cm to 183
		reachThePosition = false;
		//cout << "goig to pos:(" << xGoal << "," << yGoal << ")";
	}

	if (!(isNear(realRobotX, xGoal, 10) && isNear(realRobotY, yGoal, 10)))
	{
		int movimentAngle = angleTwoPoints(realRobotX, realRobotY, xGoal, yGoal);
		simpleMove(0, movimentAngle);
		return true;
	}
	else{
		simpleMove();//stop
		reachThePosition = true;
		return false;
	}
}
#endif