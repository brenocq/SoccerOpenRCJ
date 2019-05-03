///////////////////////////////////////////////////////////
//  Compass.cpp
//  Implementation of the Class Compass
//  Created on:      21-mar-2018 21:46:56
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Compass.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan



#if defined(ARDUINO)
Compass::Compass()
	:Sensor(0, 0),adjustFront(0)
{
	
}

void Compass::begin(){
	Serial.begin(9600);
	Serial1.begin(9600);
}

#else
Compass::Compass(string name, int robotN)
	: Sensor(name, robotN), adjustFront(0), isFirstReading(true)
{

}
#endif

Compass::~Compass(){

}

//value
float Compass::value() {
#if defined(ARDUINO)
	int compassValue=0;
	Serial1.write(0x13);
	if (Serial1.available()) {
		uint8_t _highByte = Serial1.read();
		uint8_t _lowByte = Serial1.read();

		compassValue = _highByte;// Calculate 16 bit angle
		compassValue <<= 8;
		compassValue += _lowByte;

		float f_compassValue = float(compassValue) / 10;

		f_compassValue < 0 ? f_compassValue += 360 : f_compassValue;

		return f_compassValue;
	}
	else
		return 0;
	
#else
	

	float angles[3] = { 0, 0, 0 };

	simxGetObjectOrientation(clientIDSimulation, handleSimulation, -1, (simxFloat*)angles, simx_opmode_oneshot);

	float degreeX = ((angles[2] * 180 / M_PI))*(-1) - 270 - (adjustFront);//+45 for simulation robot compansation(X axis)

	while (degreeX < 0)
		degreeX += 360;
	while (degreeX >= 360)
		degreeX -= 360;

	if (degreeX != 0 && degreeX != 90 && isFirstReading == true)//set the first value 
	{															//(the first values in simulation are wrong - 0 or 90 degree)
		adjustFront = degreeX;
		isFirstReading = false;
		degreeX = value();
	}

	return degreeX;
#endif
	//return compassValue;
}

#if defined(ARDUINO)
void Compass::updateAccel(){
	int compassValue = 0;
	Serial1.write(0x20);
	if (Serial1.available()) {
		uint8_t accelX_HighByte = Serial1.read();
		uint8_t accelX_LowByte = Serial1.read();
		uint8_t accelY_HighByte = Serial1.read();
		uint8_t accelY_LowByte = Serial1.read();
		uint8_t accelZ_HighByte = Serial1.read();
		uint8_t accelZ_LowByte = Serial1.read();

		accelerometerX = accelX_HighByte;
		accelerometerX <<= 8;
		accelerometerX += accelX_LowByte;

		accelerometerY = accelY_HighByte;
		accelerometerY <<= 8;
		accelerometerY += accelY_LowByte;

		accelerometerZ = accelZ_HighByte;
		accelerometerZ <<= 8;
		accelerometerZ += accelZ_LowByte;
	}
}
int Compass::accelX(){
	return accelerometerX;
}
int Compass::accelY(){
	return accelerometerY;
}
int Compass::accelZ(){
	return accelerometerZ;
}

void Compass::print(bool plotter){
	if (!plotter){//monitor serial
		Serial.print("Compass:");
		Serial.print(value());
		Serial.println("°");
		/*Serial.print(" ");
		Serial.print("AccelX:");
		Serial.println(accelX());*/
	}
	else{//plotter
		//Serial.print("Compass:");
		Serial.println(value());
		/*Serial.print("		AccelX:");
		Serial.println(accelX());*/
	}
}
#endif