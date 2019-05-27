// Library (Motors) robô de futebol 2018 CIC Robotics
// 05/03/2018
// Desenvolvedor: Breno Cunha Queiroz

#ifndef DRIBBLE_H
#define DRIBBLE_H

class Dribble
{
public:
	Dribble();
	void setMaxPower(int maximumPower);//(maximumPower)
	void pin(int pos, int neg, int pwm);//(pin+, pin-, pinPWM)
	void motor(float motorPower);

	//Simulation
	void initializeSimulation(int ID);//initialize the simulation motors and ID

private:
	int maxPower;

	int dribble[3];//pins [+,-,pwm]
	//Simulation
	int clientIDSimulation;
	int dribbleSimulation;

};

#endif // DRIBBLE_H