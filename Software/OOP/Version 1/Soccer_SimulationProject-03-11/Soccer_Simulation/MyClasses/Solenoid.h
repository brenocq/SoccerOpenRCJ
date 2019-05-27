// Library (Solenoid) robô de futebol 2018 CIC Robotics
// 11/03/2018
// Desenvolvedor: Breno Cunha Queiroz

#ifndef SOLENOID_H
#define SOLENOID_H


class Solenoid
{
public:
	Solenoid();
	void pins();

	void initializeSimulation(int ID);//initialize the simulation BPsensor
	void kick();

private:
	//Simulation
	int clientIDSimulation;
	int handleSolenoid;
	float solenoidnormalPos[3];
	float solenoidkickPos[3];
};

#endif // SOLENOID_H