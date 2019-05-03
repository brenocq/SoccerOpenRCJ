///////////////////////////////////////////////////////////
//  Camera.cpp
//  Implementation of the Class Camera
//  Created on:      21-mar-2018 21:46:56
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Camera.h"


#if !defined(ARDUINO)
#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;
#include <iomanip>//setprecision
using std::setprecision;
#endif

#if defined(ARDUINO)
#include <Wire.h>
Camera::Camera()
	: Sensor(0, 0), fieldOfViewH(75), fieldOfViewV(47)
{

}

void Camera::begin(){
	pixy = new PixyI2C;
	pixy->init();
}

#else
Camera::Camera(string name,int robotN)
	: Sensor(name, robotN), fieldOfViewH(75), fieldOfViewV(75)
{

}
#endif


Camera::~Camera(){

}

#if !defined(ARDUINO)
void Camera::initializeSimulation(int ID)
{
	Sensor::initializeSimulation(ID);

	string name1 = sensorName + "1#" + std::to_string(robotNumber);
	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)name1.c_str(), (simxInt *)&handleCam1, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << sensorName << "1- robot" << robotNumber << " not found!" << std::endl;
	else
		cout << "Connected to the sensor " << sensorName << "1- robot" << robotNumber << std::endl;

	string name2 = sensorName + "2#" + std::to_string(robotNumber);
	if (simxGetObjectHandle(clientIDSimulation, (const simxChar*)name2.c_str(), (simxInt *)&handleCam2, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		cout << sensorName << "2- robot" << robotNumber << " not found!" << std::endl;
	else
		cout << "Connected to the sensor " << sensorName << "2- robot" << robotNumber << std::endl;

	simxReadVisionSensor(clientIDSimulation, handleSimulation, &state, &auxCameraValues, &auxCameraValuesCount, simx_opmode_streaming);
	simxReadVisionSensor(clientIDSimulation, handleCam1, &state, &auxCameraValues, &auxCameraValuesCount, simx_opmode_streaming);
	simxReadVisionSensor(clientIDSimulation, handleCam2, &state, &auxCameraValues, &auxCameraValuesCount, simx_opmode_streaming);
}
#endif

void Camera::updateCamera(){
#if defined(ARDUINO)
	static int i = 0;
	int j;
	uint16_t blocks;
	char buf[32];

	blocks = pixy->getBlocks();
	if (blocks){
		setNumberObjects(blocks);
		for (int i = 0; i < blocks; i++){
			int size = (pixy->blocks[i].width) > (pixy->blocks[i].height) ? (pixy->blocks[i].width) : (pixy->blocks[i].height);

			float _objX = pixy->blocks[i].x;
			float _objY = pixy->blocks[i].y;

			setObjectX(_objX / 320, i);
			setObjectY(_objY / 200, i);

			if ((pixy->blocks[i].width) > (pixy->blocks[i].height)){
				float _objSize = pixy->blocks[i].width;
				setObjectSize(_objSize / 320, i);
			}
			else{
				float _objSize = pixy->blocks[i].height;
				setObjectSize(_objSize / 200, i);
			}
		}
		for (int i = blocks; i < 10; i++){
			setObjectX(0, i);
			setObjectY(0, i);
			setObjectSize(0, i);
		}
		/*Serial.print("pixy X:");
		Serial.print(pixy->blocks[0].x);
		Serial.print("  Y:");
		Serial.print(pixy->blocks[0].y);
		Serial.print("  width:");
		Serial.print(pixy->blocks[0].width);
		Serial.print("  height:");
		Serial.print(pixy->blocks[0].height);
		Serial.print("  signature:");
		Serial.println(pixy->blocks[0].signature);*/
	}
	else {
		for (int i = 0; i < 10; i++){
			setObjectX(0, i);
			setObjectY(0, i);
			setObjectSize(0, i);
			setNumberObjects(0);
		}
	}
#else
	if (simxReadVisionSensor(clientIDSimulation, handleSimulation, &state, &auxCameraValues, &auxCameraValuesCount, simx_opmode_streaming) == simx_return_ok){
		setNumberObjects(auxCameraValues[15],0);//nothing=0 (ball)
		float objectLenght = auxCameraValues[21];
		float objectHeight = auxCameraValues[22];
		if (getNumberObjects("ball") != 0) 
		{
			setObjectX(auxCameraValues[19]);
			setObjectY(auxCameraValues[20]);

			if (objectHeight > objectLenght)
				setObjectSize(objectHeight);
			else
				setObjectSize(objectLenght);
		}
		else
		{
			setNumberObjects(0);
			setObjectX(0);
			setObjectY(0);
		}
	}
	//----- Camera 1(yellow) -----//
	if (simxReadVisionSensor(clientIDSimulation, handleCam1, &state, &auxCameraValues, &auxCameraValuesCount, simx_opmode_streaming) == simx_return_ok){
		//(15)num objects (19)Xobj1 (20)Yobj1 (21)Lenghtobj1 (22)Heightobj1
		//		(23) (24) (25)Xobj2 (26)Yobj2 (27)Lenghtobj2 (28)Heightobj2
		setNumberObjects(auxCameraValues[15], 1);
		int selectObject = 0;
		if (getNumberObjects("goaly") == 2){
			if (auxCameraValues[21] < auxCameraValues[27])//if object 1 is bigger than object 2
				selectObject = 6;//use date from the other object

		}
		if (getNumberObjects("goaly") !=0)
		{
			setNumberObjects(auxCameraValues[15], 1);
			float objectLenght = auxCameraValues[21 + selectObject];
			float objectHeight = auxCameraValues[22 + selectObject];

			setObjectX(auxCameraValues[19 + selectObject], 1);
			setObjectY(auxCameraValues[20 + selectObject], 1);
			setObjectHeight(objectHeight, 1);
			setObjectLength(objectLenght, 1);

			if (objectHeight > objectLenght)
				setObjectSize(objectHeight,1);
			else
				setObjectSize(objectLenght,1);
		}
		else
		{
			setNumberObjects(0,1);
			setObjectX(0,1);
			setObjectY(0,1);
		}
	}
	//----- Camera 2(blue) -----//
	if (simxReadVisionSensor(clientIDSimulation, handleCam2, &state, &auxCameraValues, &auxCameraValuesCount, simx_opmode_streaming) == simx_return_ok){
		//(15)num objects (19)Xobj1 (20)Yobj1 (21)Lenghtobj1 (22)Heightobj1
		//		(23) (24) (25)Xobj2 (26)Yobj2 (27)Lenghtobj2 (28)Heightobj2
		setNumberObjects(auxCameraValues[15], 2);
		int selectObject = 0;
		if (getNumberObjects("goalb") == 2){
			if (auxCameraValues[21] < auxCameraValues[27])//if object 1 is bigger than object 2
				selectObject = 6;//use date from the other object

		}
		if (getNumberObjects("goalb") != 0)
		{
			setNumberObjects(auxCameraValues[15], 2);
			float objectLenght = auxCameraValues[21 + selectObject];
			float objectHeight = auxCameraValues[22 + selectObject];

			setObjectX(auxCameraValues[19 + selectObject], 2);
			setObjectY(auxCameraValues[20 + selectObject], 2);
			setObjectHeight(objectHeight, 2);
			setObjectLength(objectLenght, 2);

			if (objectHeight > objectLenght)
				setObjectSize(objectHeight, 2);
			else
				setObjectSize(objectLenght, 2);
		}
		else
		{
			setNumberObjects(0, 2);
			setObjectX(0, 2);
			setObjectY(0, 2);
		}
	}
#endif
}

void Camera :: print()const{
#if defined(ARDUINO)
	if (getNumberObjects()>0)
	{
		int numObjects = getNumberObjects();
		Serial.print("pixy numObj:");
		Serial.print(numObjects);
		for (int i = 0; i<numObjects; i++)
		{
			Serial.println(" ");
			Serial.print("    Object ");
			Serial.print(i);
			Serial.print("-->  X:");
			Serial.print(getObjectX(i));
			Serial.print("  Y:");
			Serial.print(getObjectY(i));
			Serial.print("  size:");
			Serial.print(getObjectSize(i));
			delay(100);
		}
		delay(1000);
		Serial.println(" ");
		Serial.println(" ");
	}
	else
		Serial.println("0 objects detected");
#else
#endif
}

//Get property
float Camera::getObjectX(string name)const {
	if (name=="ball")
	return objectX[0];
	else if (name == "goal"){
		if (getNumberObjects("goalb") > getNumberObjects("goaly"))
			return objectX[2];
		else
			return objectX[1];
	}
	else
		cout << endl << "getObjectX Camera: please use a valid name";
}
float Camera::getObjectY(string name)const {
	if (name == "ball")
		return objectY[0];
	else if (name == "goal"){
		if (getNumberObjects("goalb") > getNumberObjects("goaly"))
			return objectY[2];
		else
			return objectY[1];
	}
	else
		cout << endl << "getObjectY Camera: please use a valid name";
}
float Camera::getObjectSize(string name)const
{
	if (name == "ball")
		return objectSize[0];
	else if (name == "goal"){
		if (objectSize[1] > objectSize[2])
			return objectSize[1];
		else
			return objectSize[2];
	}
	else
		cout << endl << "getObjectSize Camera: please use a valid name";
}
float Camera::getObjectLength(string name)const
{
	if (name == "ball")
		return objectLenght[0];
	else if (name == "goal"){
		if (getNumberObjects("goalb") > getNumberObjects("goaly"))
			return objectLenght[2];
		else
			return objectLenght[1];
	}
	else
		cout << endl << "getObjectSize Camera: please use a valid name";
	return 0;
}
float Camera::getObjectHeight(string name)const
{
	if (name == "ball")
		return objectHeight[0];
	else if (name == "goal"){
		if (getNumberObjects("goalb") > getNumberObjects("goaly"))
			return objectHeight[2];
		else
			return objectHeight[1];
	}
	else
		cout << endl << "getObjectSize Camera: please use a valid name";
	return 0;
}

int Camera::getNumberObjects(string name) const{
	if (name == "ball")
		return numberObjects[0];
	else if (name == "goaly")
		return numberObjects[1];
	else if (name == "goalb")
		return numberObjects[2];
	else
		cout << endl << "getObjectSize Camera: please use a valid name";
	return 0;
}
int Camera::getFieldOfViewH()const
{
	return fieldOfViewH;
}
int Camera::getFieldOfViewV()const
{
	return fieldOfViewV;
}

//Set property
void Camera::setObjectX(float val, int objN){
	objectX[objN] = val;
}
void Camera::setObjectY(float val, int objN){
	objectY[objN] = val;
}
void  Camera::setObjectSize(float val, int objN)
{
	objectSize[objN] = val;
}
void  Camera::setObjectLength(float val, int objN)
{
	objectLenght[objN] = val;
}
void  Camera::setObjectHeight(float val, int objN)
{
	objectHeight[objN] = val;
}
void Camera::setNumberObjects(int val, int objN){
	numberObjects[objN] = val;
}