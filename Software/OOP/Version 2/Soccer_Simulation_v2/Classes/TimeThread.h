///////////////////////////////////////////////////////////
//  TimeThread.h
//  Implementation of the Class TimeThread
//  Created on:      21-mar-2018 21:46:59
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef TIMETHREAD_H
#define TIMETHREAD_H

class TimeThread
{

public:
	TimeThread(int time);
	virtual ~TimeThread();

	void setTimeMilliseconds(int time);
	void defineTimePoint();
	
	int getTimeMilliseconds() const;
	unsigned int getLastTime() const;
	unsigned int getCurrentTime() const;
	int getTimePoint();

	bool waitPointTime();// execute if now is X miliseconds after the timePoint
	bool executeThread();//execute every X miliseconds

	int lastDiscrepancy;
private:
	unsigned int timeMilliseconds;
	unsigned long lastTime;
	unsigned long currentTime;
	unsigned long timePoint;
	
	void update();

};
#endif // TIMETHREAD_H
