// Library (TimeThread) robô de futebol 2018 CIC Robotics
// 03/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#ifndef TIMETHREAD_H
#define TIMETHREAD_H

class TimeThread
{
public:
	TimeThread(int time);

	void update();
	bool executeThread();
	bool executePointThread();
	int getLastTime() const;
	int getCurrentTime() const;
	int getTimeMilliseconds() const;
	void setTimeMilliseconds(int newTimeMilliseconds);

	void setPointTime();
	int getPointTime();

	int lastDiscrepancy;
private:
	unsigned int timeMilliseconds;
	unsigned long lastTime;
	unsigned long currentTime;

	unsigned long pointTime;
};

#endif // TIMETHREAD_H