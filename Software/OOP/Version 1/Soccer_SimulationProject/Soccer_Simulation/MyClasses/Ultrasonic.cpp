// Library (Ultrasonic) rob๔ de futebol 2018 CIC Robotics
// 16/01/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "Ultrasonic.h"

#include "TimeThread.h"
#include "DataBase.h"
#include "Compass.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

#if defined(ARDUINO) && ARDUINO>=100
 #include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
 #include "WProgram.h"//arduino old version
#else

 #include <string>//string
 using std::string;
 #include <iostream>//cout,cin
 using std::cout;
 using std::cin;
 using std::endl;
#include <iomanip>//cout,cin
 using std::setprecision;

 extern "C" {
 #include "extApi.h"
 }
#endif

 Ultrasonic::Ultrasonic(DataBase &robotDataBase, Compass &robotCompass)
	 : dataBase(robotDataBase), compass(robotCompass)
{
	clientIDSimulation = 0;

	for (int i = 0; i < 12; i++)
	{
		sonarPointX[i] = 0;
	}
	for (int i = 0; i < 12; i++)
	{
		sonarPointY[i] = 0;
	}
	widthField = dataBase.getWidthField();
	lengthField = dataBase.getLengthField();
	isFirstRobotX = true;
	isFirstRobotY = true;
	discrepancyX = 10;
	discrepancyY = 20;

	increaseDiscrepancyX = new TimeThread(100);
	increaseDiscrepancyY = new TimeThread(100);
}

void Ultrasonic::initializeSimulation(int ID)
{
	clientIDSimulation = ID;

	#if !defined(ARDUINO)

	string sensorName[12];
	for (int i = 0; i < 12; i++)
	{
		sensorName[i] = "Sonar_" + std::to_string(i + 1);

		if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)sensorName[i].c_str(), (simxInt *)&sonarHandle[i], (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
			cout << "Sensor handle " << sensorName[i] << " not found!" << std::endl;
		else
		{
			cout << "Connected to the sensor " << sensorName[i] << std::endl;
			simxReadProximitySensor(clientIDSimulation, sonarHandle[i], NULL, NULL, NULL, NULL, simx_opmode_streaming);
		}
	}
	#endif
}

float Ultrasonic::value(int sonarNumber)
{
	float compassValue = compass.value();

	#if defined(ARDUINO)
	int spinTimes = 0;//Number of sonar spins|EX: spin=2 -> Sonar 1 turns 3/Sonar 2 turns 4
	int spinTime_auxiliary = 1;
	int RealNumberSonar;//The real number after degree tratament

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

	RealNumberSonar = sonarNumber - spinTimes;
	while (RealNumberSonar > 11)
		RealNumberSonar -= 12;
	while (RealNumberSonar < 0)
		RealNumberSonar += 12;

	//return analog(...)//remember: return (RealNumberSonar % 3) == 0 ? dist * 100 + 10 : dist * 100 + 11;
	return 0;
	#else
		int spinTimes = 0;//Number of sonar spins|EX: spin=2 -> Sonar 1 turns 3/Sonar 2 turns 4
		int spinTime_auxiliary = 1;
		int RealNumberSonar;//The real number after degree tratament

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

		RealNumberSonar = sonarNumber - spinTimes;
		while (RealNumberSonar > 11)
			RealNumberSonar -= 12;
		while (RealNumberSonar < 0)
			RealNumberSonar += 12;

		simxUChar state;
		simxFloat coord[3];

		if (simxReadProximitySensor(clientIDSimulation, sonarHandle[RealNumberSonar], &state, coord, NULL, NULL, simx_opmode_buffer) == simx_return_ok)
		{
			float dist = coord[2];
			if (state > 0)
			{
				//sonar 3,6 and 9 are inside the robot(ratio 10)
				return (RealNumberSonar % 3) == 0 ? dist * 100 + 10 : dist * 100 + 11;// 11 is the ratio of the robot
			}
		}
	#endif
}

float Ultrasonic::angle(int numberSonar, bool isRecursion)
{
	int spinTimes = 0;//Number of sonar spins|EX: spin=2 -> Sonar 1 turns 3/Sonar 2 turns 4
	int numberSonarSpin = 0;//Number of sonar the after spin
	float sonarAngleValue = 0;//will return this number
	float compassValue = compass.value();

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
	if (isRecursion == false)//before recursion
	{
		if (numberSonar == 0)
		{
			numberSonarSpin = 12 - spinTimes;
			if (spinTimes == 0)
				return Ultrasonic::angle(numberSonar, true);//recursion
			else
				return Ultrasonic::angle(numberSonarSpin, true);//recursion
		}
		else
		{
			numberSonarSpin = numberSonar - spinTimes;
			if (spinTimes == 0)
				return Ultrasonic::angle(numberSonar, true);//recursion
			else
				if (numberSonarSpin >= 0)
					return Ultrasonic::angle(numberSonarSpin, true);//recursion
				else
					return Ultrasonic::angle(12 - spinTimes + numberSonar, true);//recursion
		}
	}
	else//after recursion
	{
		float degreeError = compassValue;
		if (degreeError > 180)
		{
			degreeError = degreeError - 360;
		}

		if (degreeError <= 180)
			sonarAngleValue = degreeError - (360 - numberSonar * 30);
		else
			sonarAngleValue = degreeError + (numberSonar * 30);

		while (sonarAngleValue < 0)//SonarAngleDistortion can be negative
		{
			sonarAngleValue += 360;
		}
		while (sonarAngleValue >= 360)
		{
			sonarAngleValue -= 360;
		}
		return sonarAngleValue;
	}
}

void Ultrasonic::updatePointX()
{
	//-----------------sonar 0 analysis (345ฐ <-0ฐ-> 15ฐ)
	if (angle(0) < 180)
		sonarPointX[0] = value(0)*sin(angle(0)*M_PI / 180);
	else
		sonarPointX[0] = (-1)* value(0)*sin((360 - angle(0)) *M_PI / 180);
	//-----------------sonar 1 analysis (15ฐ <-30ฐ-> 45ฐ)
	sonarPointX[1] = value(1)*sin(angle(1)*M_PI / 180);
	//-----------------sonar 2 analysis (45ฐ <-60ฐ-> 75ฐ)
	sonarPointX[2] = value(2)*sin(angle(2)*M_PI / 180);
	//-----------------sonar 3 analysis (75ฐ <-90ฐ-> 105ฐ)
	if (angle(3) <= 90)
		sonarPointX[3] = value(3)*cos((90 - angle(3))*M_PI / 180);
	else
		sonarPointX[3] = (value(3) * cos((angle(3) - 90)*M_PI / 180));
	//-----------------sonar 4 analysis (105ฐ <-120ฐ-> 135ฐ)
	sonarPointX[4] = (value(4) * cos((angle(4) - 90) *M_PI / 180));
	//-----------------sonar 5 analysis (135ฐ <-150ฐ-> 165ฐ)
	sonarPointX[5] = (value(5) * cos((angle(5) - 90) *M_PI / 180));
	//-----------------sonar 6 analysis(165ฐ <-180ฐ-> 195ฐ)
	if (angle(6) < 180)
		sonarPointX[6] = value(6)*sin((180 - angle(6)) *M_PI / 180);
	else
		sonarPointX[6] = (-1)* value(6)*sin((angle(6) - 180) *M_PI / 180);
	//-----------------sonar 7 analysis(195ฐ <-210ฐ-> 225ฐ)
	sonarPointX[7] = (-1)*(value(7)) * cos(((360 - angle(7)) - 90)*M_PI / 180);
	//-----------------sonar 8 analysis(225ฐ <-240ฐ-> 255ฐ)
	sonarPointX[8] = (-1)*(value(8)) * cos(((360 - angle(8)) - 90)*M_PI / 180);
	//-----------------sonar 9 analysis(255ฐ <-270ฐ-> 285ฐ)
	if (angle(9) <= 270)
		sonarPointX[9] = (-1)* (value(9)) * cos(((360 - angle(9)) - 90)*M_PI / 180);
	else
		sonarPointX[9] = (-1)* (value(9)) * cos((angle(9) - 270)*M_PI / 180);
	//-----------------sonar 10 analysis(285ฐ <-300ฐ-> 315ฐ)
	sonarPointX[10] = (-1)* (value(10)) * cos((angle(10) - 270)*M_PI / 180);
	//-----------------sonar 11 analysis(315ฐ <-330ฐ-> 345ฐ)
	sonarPointX[11] = (-1)* (value(11)) * cos((angle(11) - 270)*M_PI / 180);

}

void Ultrasonic::updatePointY()
{
	//-----------------sonar 0 analysis (345ฐ <-0ฐ-> 15ฐ)
	if (angle(0) < 180)
		sonarPointY[0] = value(0)*cos(angle(0)*M_PI / 180);
	else
		sonarPointY[0] = value(0)*cos((360 - angle(0)) *M_PI / 180);
	//-----------------sonar 1 analysis (15ฐ <-30ฐ-> 45ฐ)
	sonarPointY[1] = value(1)*cos(angle(1)*M_PI / 180);
	//-----------------sonar 2 analysis (45ฐ <-60ฐ-> 75ฐ)
	sonarPointY[2] = value(2)*cos(angle(2)*M_PI / 180);
	//-----------------sonar 3 analysis (75ฐ <-90ฐ-> 105ฐ)
	if (angle(3) <= 90)
		sonarPointY[3] = value(3)*sin((90 - angle(3))*M_PI / 180);
	else
		sonarPointY[3] = (-1)*(value(3) * sin((angle(3) - 90)*M_PI / 180));
	//-----------------sonar 4 analysis (105ฐ <-120ฐ-> 135ฐ)
	sonarPointY[4] = (-1)* (value(4) * sin((angle(4) - 90) *M_PI / 180));
	//-----------------sonar 5 analysis (135ฐ <-150ฐ-> 165ฐ)
	sonarPointY[5] = (-1)* (value(5) * sin((angle(5) - 90) *M_PI / 180));
	//-----------------sonar 6 analysis(165ฐ <-180ฐ-> 195ฐ)
	if (angle(6) < 180)
		sonarPointY[6] = (-1)* value(6)*cos((180 - angle(6)) *M_PI / 180);
	else
		sonarPointY[6] = (-1)* value(6)*cos((angle(6) - 180) *M_PI / 180);
	//-----------------sonar 7 analysis(195ฐ <-210ฐ-> 225ฐ)
	sonarPointY[7] = (-1)* (value(7)) * sin(((360 - angle(7)) - 90)*M_PI / 180);
	//-----------------sonar 8 analysis(225ฐ <-240ฐ-> 255ฐ)
	sonarPointY[8] = (-1)* (value(8)) * sin(((360 - angle(8)) - 90)*M_PI / 180);
	//-----------------sonar 9 analysis(255ฐ <-270ฐ-> 285ฐ)
	if (angle(9) <= 270)
		sonarPointY[9] = (-1)* (value(9)) * sin(((360 - angle(9)) - 90)*M_PI / 180);
	else
		sonarPointY[9] = (value(9)) * sin((angle(9) - 270)*M_PI / 180);
	//-----------------sonar 10 analysis(285ฐ <-300ฐ-> 315ฐ)
	sonarPointY[10] = (value(10)) * sin((angle(10) - 270)*M_PI / 180);
	//-----------------sonar 11 analysis(315ฐ <-330ฐ-> 345ฐ)
	sonarPointY[11] = (value(11)) * sin((angle(11) - 270)*M_PI / 180);
}

int Ultrasonic::getPointX(int sonarNumber)
{
	return sonarPointX[sonarNumber];
}

int Ultrasonic::getPointY(int sonarNumber)
{
	return sonarPointY[sonarNumber];
}

int Ultrasonic::getRobotX()
{
	updatePointX();

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


	if (isFirstRobotX == false)
	{
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				if (isNear(possibleValues[i], possibleValues[j], 4) && isNear(possibleValues[i], dataBase.getRobotX(), discrepancyX))
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
		dataBase.setIsRobotXKnowed(true);//Know X

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
			dataBase.setRobotX(possibleX);
			isFirstRobotX = false;
		}


		if (!isNear(possibleX, dataBase.getRobotX(), discrepancyX))
		{
			//cout << possibleX << " is more than " << discrepancyTime << " distant of " << lastRobotX << "	:(" << endl;
			dataBase.setIsRobotXKnowed(false);//dont know X
		}
		else
		{
			discrepancyX = 10;
			dataBase.setIsRobotXKnowed(true);
		}
		if (dataBase.getIsRobotXKnowed() == true)
		{
			dataBase.setRobotX(possibleX);
		}
	}
	else
		dataBase.setIsRobotXKnowed(false);//dont know X

	if (dataBase.getIsRobotXKnowed() == false)
	{
		//cout << "I don't know the position X value" << endl;
		increaseDiscrepancyX->setTimeMilliseconds(100);
		increaseDiscrepancyX->update();
		if (increaseDiscrepancyX->executeThread() == true)
		{
			int lostCycles = (increaseDiscrepancyX->lastDiscrepancy) / 100;
			discrepancyX += 5 * lostCycles;
		}
	}

	return dataBase.getRobotX();
}

int Ultrasonic::getRobotY()
{
	int robotX = dataBase.getRobotX();

	updatePointY();
	updatePointX();

	int possibleValues[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float ValuesProbability[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//probability 0 to 1

	float  ProbabilityGrowing[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//probability growing
	int SonarIndexProbabilityGrowing[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//sonar index of probability growing

	//COMEวAR A ANALISAR DO ฺLTIMO, SE ANALISAR COM O ULTIMO VALOR E FOR MUITO DISCREPANTE AI PASSA PARA O PRำXIMO
	//SE ENCONTRAR UM QUE SATISFAZ A CONDIวรO Jม ษ O robotY
	//SE NรO ENCONTRAR NENHUM ENTรO DECRETA isYKnowed=false

	int possibleY = 0;//More probable
	//-----------------sonar 0 analysis
	if ((robotX + sonarPointX[0] > widthField - 5) || (robotX + sonarPointX[0] <5))
		possibleValues[0] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[0] <(widthField / 2) + 33) && (robotX + sonarPointX[0] >(widthField / 2) + 27))
		possibleValues[0] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[0] <(widthField / 2) - 27) && (robotX + sonarPointX[0] >(widthField / 2) - 33))
		possibleValues[0] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[0] > (widthField / 2) + 30) || (robotX + sonarPointX[0] < (widthField / 2) - 30))
		possibleValues[0] = lengthField - sonarPointY[0];
	else
		possibleValues[0] = lengthField - ((30 - 7.5) + sonarPointY[0]);
	//-----------------sonar 1 analysis
	if ((robotX + sonarPointX[1] > widthField - 5) || (robotX + sonarPointX[1] <5))
		possibleValues[1] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[1] <(widthField / 2) + 33) && (robotX + sonarPointX[1] >(widthField / 2) + 27))
		possibleValues[1] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[1] <(widthField / 2) - 27) && (robotX + sonarPointX[1] >(widthField / 2) - 33))
		possibleValues[1] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[1] > (widthField / 2) + 30) || (robotX + sonarPointX[1] < (widthField / 2) - 30))
		possibleValues[1] = lengthField - sonarPointY[1];
	else
		possibleValues[1] = lengthField - (30 - 7.5) - sonarPointY[1];
	//-----------------sonar 2 analysis
	if ((robotX + sonarPointX[2] > widthField - 5) || (robotX + sonarPointX[2] <5))
		possibleValues[2] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[2] <(widthField / 2) + 33) && (robotX + sonarPointX[2] >(widthField / 2) + 27))
		possibleValues[2] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[2] <(widthField / 2) - 27) && (robotX + sonarPointX[2] >(widthField / 2) - 33))
		possibleValues[2] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[2] > (widthField / 2) + 30) || (robotX + sonarPointX[2] < (widthField / 2) - 30))
		possibleValues[2] = lengthField - sonarPointY[2];
	else
		possibleValues[2] = lengthField - (30 - 7.5) - sonarPointY[2];
	//-----------------sonar 4 analysis
	if ((robotX + sonarPointX[4] > widthField - 5) || (robotX + sonarPointX[4] <5))
		possibleValues[3] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[4] <(widthField / 2) + 33) && (robotX + sonarPointX[4] >(widthField / 2) + 27))
		possibleValues[3] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[4] <(widthField / 2) - 27) && (robotX + sonarPointX[4] >(widthField / 2) - 33))
		possibleValues[3] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[4] > (widthField / 2) + 30) || (robotX + sonarPointX[4] < (widthField / 2) - 30))
		possibleValues[3] = (-1)*sonarPointY[4];//sonarPointY is negative
	else
		possibleValues[3] = (30 - 7.5) + (-1)*sonarPointY[4];//sonarPointY is negative
	//-----------------sonar 5 analysis
	if ((robotX + sonarPointX[5] > widthField - 5) || (robotX + sonarPointX[5] <5))
		possibleValues[4] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[5] <(widthField / 2) + 33) && (robotX + sonarPointX[5] >(widthField / 2) + 27))
		possibleValues[4] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[5] <(widthField / 2) - 27) && (robotX + sonarPointX[5] >(widthField / 2) - 33))
		possibleValues[4] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[5] > (widthField / 2) + 30) || (robotX + sonarPointX[5] < (widthField / 2) - 30))
		possibleValues[4] = (-1)*sonarPointY[5];//sonarPointY is negative
	else
		possibleValues[4] = (30 - 7.5) + (-1)*sonarPointY[5];//sonarPointY is negative
	//-----------------sonar 6 analysis
	if ((robotX + sonarPointX[6] > widthField - 5) || (robotX + sonarPointX[6] <5))
		possibleValues[5] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[6] <(widthField / 2) + 33) && (robotX + sonarPointX[6] >(widthField / 2) + 27))
		possibleValues[5] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[6] <(widthField / 2) - 27) && (robotX + sonarPointX[6] >(widthField / 2) - 33))
		possibleValues[5] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[6] > (widthField / 2) + 30) || (robotX + sonarPointX[6] < (widthField / 2) - 30))
		possibleValues[5] = (-1)*sonarPointY[6];//sonarPointY is negative
	else
		possibleValues[5] = (30 - 7.5) + (-1)*sonarPointY[6];//sonarPointY is negative
	//-----------------sonar 7 analysis
	if ((robotX + sonarPointX[7] > widthField - 5) || (robotX + sonarPointX[7] <5))
		possibleValues[6] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[7] <(widthField / 2) + 33) && (robotX + sonarPointX[7] >(widthField / 2) + 27))
		possibleValues[6] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[7] <(widthField / 2) - 27) && (robotX + sonarPointX[7] >(widthField / 2) - 33))
		possibleValues[6] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[7] > (widthField / 2) + 30) || (robotX + sonarPointX[7] < (widthField / 2) - 30))
		possibleValues[6] = (-1)*sonarPointY[7];//sonarPointY is negative
	else
		possibleValues[6] = (30 - 7.5) + (-1)*sonarPointY[7];//sonarPointY is negative
	//-----------------sonar 8 analysis
	if ((robotX + sonarPointX[8] > widthField - 5) || (robotX + sonarPointX[8] <5))
		possibleValues[7] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[8] <(widthField / 2) + 33) && (robotX + sonarPointX[8] >(widthField / 2) + 27))
		possibleValues[7] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[8] <(widthField / 2) - 27) && (robotX + sonarPointX[8] >(widthField / 2) - 33))
		possibleValues[7] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[8] > (widthField / 2) + 30) || (robotX + sonarPointX[8] < (widthField / 2) - 30))
		possibleValues[7] = (-1)*sonarPointY[8];//sonarPointY is negative
	else
		possibleValues[7] = (30 - 7.5) + (-1)*sonarPointY[8];//sonarPointY is negative
	//-----------------sonar 10 analysis
	if ((robotX + sonarPointX[10] > widthField - 5) || (robotX + sonarPointX[10] <5))
		possibleValues[8] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[10] <(widthField / 2) + 33) && (robotX + sonarPointX[10] >(widthField / 2) + 27))
		possibleValues[8] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[10] <(widthField / 2) - 27) && (robotX + sonarPointX[10] >(widthField / 2) - 33))
		possibleValues[8] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[10] > (widthField / 2) + 30) || (robotX + sonarPointX[10] < (widthField / 2) - 30))
		possibleValues[8] = lengthField - sonarPointY[10];
	else
		possibleValues[8] = lengthField - ((30 - 7.5) + sonarPointY[10]);
	//-----------------sonar 11 analysis
	if ((robotX + sonarPointX[11] > widthField - 5) || (robotX + sonarPointX[11] <5))
		possibleValues[9] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[11] <(widthField / 2) + 33) && (robotX + sonarPointX[11] >(widthField / 2) + 27))
		possibleValues[9] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[11] <(widthField / 2) - 27) && (robotX + sonarPointX[11] >(widthField / 2) - 33))
		possibleValues[9] = 0;//ignore this value (probabily is a lateral wall)
	else if ((robotX + sonarPointX[11] > (widthField / 2) + 30) || (robotX + sonarPointX[11] < (widthField / 2) - 30))
		possibleValues[9] = lengthField - sonarPointY[11];
	else
		possibleValues[9] = lengthField - ((30 - 7.5) + sonarPointY[11]);

	for (int i = 0; i < 10; i++)//front sensors probability
	{
		if (possibleValues[i] != 0)
		{
			ValuesProbability[i] = 1 - (float(possibleValues[i]) / float(lengthField));
		}

		if (i == 2)
			i = 7;
	}
	for (int i = 3; i < 8; i++)//back sensors probability
	{
		if (possibleValues[i] != 0)
		{
			ValuesProbability[i] = 1 - float(possibleValues[i]) / float(lengthField);
		}
	}

	for (int i = 0; i < 10; i++)
	{
		ProbabilityGrowing[i] = ValuesProbability[i];
		SonarIndexProbabilityGrowing[i] = i;
	}
	for (int i = 0; i < 10; i++)//ProbabilityGrowing and SonarIndexProbabilityGrowing
	{
		for (int j = i + 1; j<10; j++)
		{

			if (ProbabilityGrowing[i]>ProbabilityGrowing[j]) // mude isso aqui no seu codigo ao inves de colocar while use dois for
			{
				float tempProbability = ProbabilityGrowing[i];
				int tempIndex = SonarIndexProbabilityGrowing[i];
				ProbabilityGrowing[i] = ProbabilityGrowing[j];
				SonarIndexProbabilityGrowing[i] = SonarIndexProbabilityGrowing[j];
				ProbabilityGrowing[j] = tempProbability;
				SonarIndexProbabilityGrowing[j] = tempIndex;
			}
		}
	}

	if (isFirstRobotY == true)
	{
		dataBase.setRobotY(possibleValues[SonarIndexProbabilityGrowing[9]]);
		isFirstRobotY = false;
	}

	/*cout << setprecision(2)<< " [";
	for (int i = 9; i >= 0; i--)
	{
		cout<<possibleValues[SonarIndexProbabilityGrowing[i]];
		cout << "(" << ProbabilityGrowing[i] << ")";
		cout << ",";
	}
	cout << "]" << dataBase.getRobotY() << endl;*/

	bool updatedY = false;
	int cont1 = 9;
	dataBase.setIsRobotYKnowed(false);//if the robot know will change
	while (cont1 >= 0 && updatedY == false)
	{
		if ((possibleValues[SonarIndexProbabilityGrowing[cont1]] > 0) && (updatedY == false))
		{
			if (isNear(possibleValues[SonarIndexProbabilityGrowing[cont1]], dataBase.getRobotY(), discrepancyY))
			{
				int possibleYSum=0;//sum of the value
				int possibleYSumNumbers=0;//numbers of sonars in the sum
				float bestSonarProbability = ProbabilityGrowing[cont1];
				for (int i = 9; i >= 0; i--)
				{
					float nextSonarProbability = ProbabilityGrowing[i];
					if (isNear(bestSonarProbability, nextSonarProbability, 0.05))
					{
						possibleYSum += possibleValues[SonarIndexProbabilityGrowing[i]];
						possibleYSumNumbers++;
					}
				}
				dataBase.setRobotY(possibleYSum / possibleYSumNumbers);

				discrepancyY = 20;
				updatedY = true;
				dataBase.setIsRobotYKnowed(true);
			}
		}
		else
		{
			updatedY = true;
		}
		cont1--;
		if (cont1 < 0)
		{
			updatedY = true;
		}
	}
	if (dataBase.getIsRobotYKnowed() == false)
	{
		//cout << "I don't know the position Y value" << endl;
		increaseDiscrepancyY->setTimeMilliseconds(100);
		increaseDiscrepancyY->update();
		if (increaseDiscrepancyY->executeThread() == true)
		{
			int lostCycles = (increaseDiscrepancyY->lastDiscrepancy) / 100;
			discrepancyY += 5 * lostCycles;
		}
	}

	return dataBase.getRobotY();
}

bool Ultrasonic::getIsXknowed()
{
	return dataBase.getIsRobotXKnowed();
}

bool Ultrasonic::getIsYknowed()
{
	return dataBase.getIsRobotYKnowed();
}

void Ultrasonic::updateObstacleReading()
{
	updatePointX();
	updatePointY();

}

void Ultrasonic::updateAdversaryXY()
{

}

//private
bool Ultrasonic::isNear(float firstNumber, float secondNumber, float discrepancy)//return false if discrepancy > distance of the numbers
{
	bool isNear = true;

	if (firstNumber - secondNumber > discrepancy)
		isNear = false;
	else if (secondNumber - firstNumber > discrepancy)
		isNear = false;

	//cout << "Near(1N:" << firstNumber << ", 2N:" << secondNumber << ", maxDistance:" << discrepancy << ")= " << isNear << "\n";
	return isNear;
}

void Ultrasonic::risingVector(int vector[], int length)
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