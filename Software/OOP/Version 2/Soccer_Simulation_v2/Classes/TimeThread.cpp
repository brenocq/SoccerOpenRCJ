///////////////////////////////////////////////////////////
//  TimeThread.cpp
//  Implementation of the Class TimeThread
//  Created on:      21-mar-2018 21:46:59
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "TimeThread.h"

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else

#include <time.h>
#endif

TimeThread::TimeThread(int time)
	: timeMilliseconds(time),
	lastDiscrepancy(0)
{
#if defined(ARDUINO)
	lastTime = millis();
	currentTime = millis();
	timePoint = millis();
#else
	lastTime = clock();
	currentTime = clock();
	timePoint = clock();
#endif
}

TimeThread::~TimeThread(){

}


void TimeThread::setTimeMilliseconds(int time){
	timeMilliseconds = time;
}

void TimeThread::defineTimePoint(){
#if defined(ARDUINO)
	timePoint = millis();
#else
	timePoint = clock();
#endif
}


int TimeThread::getTimeMilliseconds()const{
	return timeMilliseconds;
}

unsigned int TimeThread::getLastTime() const{
	return lastTime;
}

unsigned int TimeThread::getCurrentTime() const{
	return currentTime;
}

int TimeThread::getTimePoint(){
	update();
	return currentTime - timePoint;
}

bool TimeThread::waitPointTime(){
	update();
	if (currentTime - timePoint >= timeMilliseconds)
	{
		return true;
	}
	else
		return false;
}

bool TimeThread::executeThread(){
	update();
	if (currentTime - lastTime >= timeMilliseconds)
	{
		lastDiscrepancy = currentTime - lastTime;
		lastTime = currentTime;
		return true;
	}
	else
		return false;
}

void TimeThread::update(){
#if defined(ARDUINO)
	currentTime = millis();
#else
	currentTime = clock();
#endif
}