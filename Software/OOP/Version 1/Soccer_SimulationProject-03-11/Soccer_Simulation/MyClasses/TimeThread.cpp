// Library (TimeThread) robô de futebol 2018 CIC Robotics
// 03/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "TimeThread.h"

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else

#include <time.h>
#endif


TimeThread::TimeThread(int time=0)
	: timeMilliseconds(time),
	lastDiscrepancy(0)
{
#if !defined(ARDUINO)
	lastTime = clock();
	currentTime = clock();
	pointTime = clock();
#else
	lastTime = millis();
	currentTime = millis();
	pointTime = millis();
#endif
}

void TimeThread::update()
{
	#if !defined(ARDUINO)
	currentTime = clock();
	#else
	currentTime = millis();
	#endif
}

bool TimeThread::executeThread()
{
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

bool TimeThread::executePointThread()
{
	update();
	if (currentTime - pointTime >= timeMilliseconds)
	{
		return true;
	}
	else
		return false;
}

int  TimeThread::getLastTime() const
{
	return lastTime;
}

int  TimeThread::getCurrentTime() const
{
	return currentTime;
}

int  TimeThread::getTimeMilliseconds() const
{
	return timeMilliseconds;
}

void  TimeThread::setTimeMilliseconds(int newTimeMilliseconds)
{
	timeMilliseconds = newTimeMilliseconds;
}

void TimeThread::setPointTime()
{
#if !defined(ARDUINO)
	pointTime = clock();
#else
	pointTime = millis();
#endif
}

int TimeThread::getPointTime()
{
	update();
	return currentTime - pointTime;
}