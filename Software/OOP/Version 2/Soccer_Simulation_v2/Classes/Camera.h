///////////////////////////////////////////////////////////
//  Camera.h
//  Implementation of the Class Camera
//  Created on:      21-mar-2018 21:46:56
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef CAMERA_H
#define CAMERA_H

#include "Sensor.h"

#if defined(ARDUINO)
#include <PixyI2C.h>
	#if ARDUINO>=100
		#include "Arduino.h"//arduino version >= 1.0.0
	#elif ARDUINO<100
		#include "WProgram.h"//arduino old version
	#endif
#else
#include <string.h>

extern "C" {
#include "../CopelliaSim/extApi.h"
}
#endif

class Camera : public Sensor
{

public:
#if defined(ARDUINO)
	Camera();

	void begin();
#else
	Camera(string name,int robotN);
#endif
	virtual ~Camera();

	

	void updateCamera();
	void print()const;

	float value(){ return 0; };

	//Get property
	float getObjectX(string name)const;
	float getObjectY(string name)const;
	float getObjectSize(string name)const;
	float getObjectHeight(string name)const;
	float getObjectLength(string name)const;
	int getNumberObjects(string name)const;
	int getFieldOfViewH()const;
	int getFieldOfViewV()const;

#if !defined(ARDUINO)
	void initializeSimulation(int ID);
#endif
private:
	//Set property
	void setNumberObjects(int val, int objN = 0);
	void setObjectX(float val, int objN=0);
	void setObjectY(float val, int objN = 0);
	void setObjectSize(float val, int objN = 0);
	void setObjectHeight(float val, int objN = 0);
	void setObjectLength(float val, int objN = 0);

	float objectX[3];
	float objectY[3];
	float objectHeight[3];
	float objectLenght[3];
	float objectSize[3];
	int numberObjects[3];

	const int fieldOfViewH;
	const int fieldOfViewV;
#if defined(ARDUINO)
	PixyI2C *pixy;


#else
	//Simulation
	int handleCam1;//cam used to detect the goal
	int handleCam2;//cam used to detect the goal
	float* auxCameraValues;//cam auxiliary
	int* auxCameraValuesCount;//cam auxiliary
	simxUChar state;
#endif

};
#endif // CAMERA_H
